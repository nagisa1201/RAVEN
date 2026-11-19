#!/usr/bin/env python3
"""RAVEN 单文件前台启动器（Python 3 标准库，无需 pip、宿主 tmux）。

./launch_raven_console.py --env RetroNeighborhood --task 'water tower'
./launch_raven_console.py --env FireAcademy --task 'fire truck' --lvlm
./launch_raven_console.py --env RetroNeighborhood --task 'water tower' --dry-run
./launch_raven_console.py --task 'water tower' --auto-takeoff --auto-plan
./launch_raven_console.py --task 'house' --low-vram --auto-takeoff --auto-plan

已有服务运行时默认拒绝启动；--restart 明确停止旧仿真并重新开始。
Ctrl+C 停止本次管理的容器，不删除镜像、卷或缓存。
默认手动起飞/跟随路径；仅显式开启 --auto-takeoff / --auto-plan 时自动发送启动指令。
实验模式：--episode-seconds 600 --goal-radius 3（本地验证参数，非已确认的论文原值）。
导航接近航点阈值：--waypoint-radius 5（默认）；设为 3 可恢复原阈值，独立于评分半径。
实验模式完成、预算耗尽或关键进程失败后保存结果并停止本轮容器。
--disable-macvo 可停用当前仿真控制未使用的视觉里程计。
默认不启动会与 RayFronts 竞争 /global_plan 的随机规划器；--keep-random-walk 恢复。
此启动器不能保证任意长时运行均无 CUDA OOM。
"""

import argparse
import concurrent.futures
import datetime
import fcntl
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parent
STACK = ROOT / 'AirStack'
RAY = 'rayfronts_container'
LVLM = 'lvlm_container'
ANSI = re.compile(r'\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))')

# Passed as a single argv item, never interpolated into shell source.
# The original input_prompt.py publishes the same String at 5 Hz.
ROS_HELPER = r'''
import os, time, rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('raven_console_task', enable_rosout=False)
task = os.environ['RAVEN_TASK']
pub = node.create_publisher(String, '/input_prompt', 10)
def publish():
    msg = String(); msg.data = task; pub.publish(msg)
node.create_timer(0.2, publish)
seen = {}
started = time.monotonic()
clock = [None, None]
def clock_cb(msg):
    value = msg.clock.sec + msg.clock.nanosec / 1e9
    if value != clock[0]:
        seen['clock'] = time.monotonic()
    clock[0] = value
def mark(key):
    return lambda msg: seen.update({key: time.monotonic()})
node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data)
node.create_subscription(Odometry, '/robot_1/odometry_conversion/odometry', mark('odom'), qos_profile_sensor_data)
node.create_subscription(PointCloud2, '/rayfronts/voxel_rgb', mark('map'), qos_profile_sensor_data)
def report():
    now = time.monotonic()
    details = []
    bad = []
    for key, limit in [('clock', 10), ('odom', 10), ('map', 60)]:
        age = now - seen.get(key, started)
        state = ('%.1fs ago' % age) if key in seen else 'waiting %.0fs' % age
        details.append(key + '=' + state)
        if (key in seen and age > limit) or (key not in seen and age > 300):
            bad.append(key)
    if clock[1] is not None and clock[0] is not None:
        old_wall, old_sim = clock[1]
        details.append('RTF=%.2f' % ((clock[0] - old_sim) / (now - old_wall)))
    if clock[0] is not None:
        clock[1] = (now, clock[0])
    if bad:
        print('WARNING 数据未更新: ' + ','.join(bad) + ' | ' + ' '.join(details), flush=True)
    else:
        print('DATA ' + ' '.join(details) + ' (进程运行/数据到达不等于任务完成)', flush=True)
node.create_timer(10.0, report)
print('TASK 已持续发布到 /input_prompt (5 Hz): ' + repr(task), flush=True)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
'''


# Runs inside the robot container, which has the custom behavior_tree_msgs package.
# Commands use the same topic/status messages as the RQT buttons, never direct
# MAVROS arming calls. Once completed/cancelled, this helper never re-arms.
AUTO_HELPER = r'''
import json, os, time, rclpy
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import State, SysStatus, ExtendedState, StatusText
from behavior_tree_msgs.msg import BehaviorTreeCommand, BehaviorTreeCommands, Status
from rclpy.qos import qos_profile_sensor_data

class AutoStart:
    # Wall seconds: simulation can run slower than real time while loading.
    PREARM_CHECK = 1 << 28
    MAX_ATTEMPTS = 3
    ARM_TIMEOUT = 30.0
    RETRY_DELAY = 10.0
    ACK_TIMEOUT = 15.0
    STATE_SYNC_TIMEOUT = 15.0
    STABLE_SECONDS = 3.0
    MANUAL_MODES = ('pause_commanded', 'land_commanded', 'disarm_commanded',
                    'rewind_commanded', 'fixed_trajectory_commanded',
                    'autonomously_explore_commanded')
    MANUAL_START_MODES = ('arm_commanded', 'takeoff_commanded', 'offboard_commanded')

    def __init__(self, node):
        self.node = node
        self.takeoff = os.environ['RAVEN_AUTO_TAKEOFF'] == '1'
        self.plan = os.environ['RAVEN_AUTO_PLAN'] == '1'
        self.timeout = float(os.environ['RAVEN_AUTO_TIMEOUT'])
        self.phase = 'WAIT_READY' if self.takeoff else 'WAIT_TAKEOFF'
        self.phase_start = time.monotonic()
        self.last_report = 0.0
        self.ready_since = None
        self.sent_at = None
        self.attempts = 0
        self.owns_takeoff = False
        self.ever_armed_or_airborne = False
        self.arm_deadline = None
        self.retry_at = None
        self.last_fcu_warning = ''
        self.seen = {}
        self.values = {}
        self.clock_value = None
        self.pub = node.create_publisher(BehaviorTreeCommands, '/robot_1/behavior/behavior_tree_commands', 10)
        node.create_subscription(Clock, '/clock', self.clock_cb, qos_profile_sensor_data)
        node.create_subscription(State, '/robot_1/interface/mavros/state',
                                 lambda msg: self.record('state', msg), qos_profile_sensor_data)
        node.create_subscription(SysStatus, '/robot_1/interface/mavros/sys_status',
                                 lambda msg: self.record('health', msg), qos_profile_sensor_data)
        node.create_subscription(ExtendedState, '/robot_1/interface/mavros/extended_state',
                                 lambda msg: self.record('landed', msg.landed_state), qos_profile_sensor_data)
        node.create_subscription(StatusText, '/robot_1/interface/mavros/statustext/recv',
                                 self.status_text_cb, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/robot_1/odometry_conversion/odometry',
                                 lambda msg: self.record('odom', True), qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/rayfronts/voxel_rgb',
                                 lambda msg: self.record('map', msg.width * msg.height > 0), qos_profile_sensor_data)
        node.create_subscription(Path, '/robot_1/global_plan',
                                 lambda msg: self.record('path', len(msg.poses) > 1), qos_profile_sensor_data)
        def bool_callback(key):
            # A single-argument callback: rclpy supplies MessageInfo to callbacks
            # with two parameters, so don't capture key as a default parameter.
            def callback(msg):
                self.record(key, msg.data)
            return callback
        for key in ('auto_takeoff_commanded', 'takeoff_complete', 'global_plan_commanded',
                    'state_estimate_timed_out') + self.MANUAL_MODES + self.MANUAL_START_MODES:
            node.create_subscription(Bool, '/robot_1/behavior/' + key + '_success',
                                     bool_callback(key), 10)
        node.create_subscription(String, '/robot_1/behavior/active_actions',
                                 lambda msg: self.record('actions', msg.data), 10)
        print('AUTO flags: takeoff=%s plan=%s; starting %s' % (self.takeoff, self.plan, self.phase), flush=True)
        node.create_timer(0.2, self.tick)

    def record(self, key, value):
        previous = self.values.get(key)
        self.values[key] = value
        self.seen[key] = time.monotonic()
        if ((key == 'state' and value.armed) or
                (key == 'landed' and value in (ExtendedState.LANDED_STATE_IN_AIR,
                                              ExtendedState.LANDED_STATE_TAKEOFF,
                                              ExtendedState.LANDED_STATE_LANDING))):
            # Latched for the lifetime of this helper, including a later landing.
            self.ever_armed_or_airborne = True
        if key == 'state':
            current_flags = (value.connected, value.armed, value.mode)
            old_flags = ((previous.connected, previous.armed, previous.mode)
                         if previous is not None else None)
            if current_flags != old_flags:
                print('AUTO FCU state changed: ' + self.state_snapshot(), flush=True)
        elif key == 'landed' and value != previous:
            print('AUTO FCU landed changed: ' + self.state_snapshot(), flush=True)

    def state_snapshot(self):
        now = time.monotonic()
        state = self.values.get('state')
        age = lambda key: '%.2fs' % (now - self.seen[key]) if key in self.seen else 'missing'
        return 'connected=%s armed=%s mode=%s state_age=%s landed=%s landed_age=%s retry_locked=%s' % (
            state.connected if state else 'unknown', state.armed if state else 'unknown',
            state.mode if state else 'unknown', age('state'), self.values.get('landed', 'unknown'),
            age('landed'), self.ever_armed_or_airborne)

    def status_text_cb(self, msg):
        if msg.severity <= StatusText.WARNING:
            warning = msg.text.strip()
            if warning and warning != self.last_fcu_warning:
                print('AUTO FCU warning: ' + warning, flush=True)
            self.last_fcu_warning = warning

    def clock_cb(self, msg):
        value = (msg.clock.sec, msg.clock.nanosec)
        if value != self.clock_value:
            self.record('clock', True)
        self.clock_value = value

    def fresh(self, key, seconds=5):
        return key in self.seen and time.monotonic() - self.seen[key] < seconds

    def truth(self, key, seconds=5):
        return self.fresh(key, seconds) and bool(self.values.get(key))

    def ready(self):
        return (self.truth('clock') and self.truth('odom') and self.fresh('state')
                and self.values['state'].connected
                and self.fresh('state_estimate_timed_out')
                and not self.values['state_estimate_timed_out']
                and self.pub.get_subscription_count() > 0)

    def airborne(self):
        return (self.ready() and self.values['state'].armed
                and self.values['state'].mode == 'OFFBOARD' and self.truth('takeoff_complete')
                and self.fresh('landed') and self.values['landed'] == ExtendedState.LANDED_STATE_IN_AIR)

    def ground_unarmed(self):
        return (self.ready() and not self.values['state'].armed and self.fresh('landed')
                and self.values['landed'] == ExtendedState.LANDED_STATE_ON_GROUND)

    def prearm_ready(self):
        # PX4 v1.16.1 SYS_STATUS.hpp sets PREARM_CHECK in health only, using
        # can_arm_mode_flags for the current mode; present/enabled omit this bit.
        return self.fresh('health') and bool(self.values['health'].sensors_health & self.PREARM_CHECK)

    def wait_reason(self):
        if not self.ready():
            return 'waiting for fresh clock/odometry/FCU/behavior-tree connection'
        if not self.fresh('landed'):
            return 'waiting for fresh FCU extended_state (landed state unknown)'
        if not self.fresh('health'):
            return 'waiting for fresh FCU sys_status (pre-arm health unknown)'
        if not self.prearm_ready() and not self.values['state'].armed:
            return 'PX4 pre-arm checks not passed'
        return 'waiting for ' + self.phase

    def fail(self, reason):
        detail = '%s; phase=%s; attempts=%d/%d; %s' % (
            reason, self.phase, self.attempts, self.MAX_ATTEMPTS, self.wait_reason())
        detail += '; ' + self.state_snapshot()
        if self.last_fcu_warning:
            detail += '; last FCU warning: ' + self.last_fcu_warning
        raise RuntimeError(detail)

    def move(self, phase):
        self.phase = phase
        self.phase_start = time.monotonic()
        print('AUTO -> ' + phase, flush=True)

    def send(self, selected):
        msg = BehaviorTreeCommands()
        for name in ('Auto Takeoff Commanded', 'Fixed Trajectory Commanded', 'Global Plan Commanded',
                     'Pause Commanded', 'Rewind Commanded', 'Disarm Commanded', 'Land Commanded',
                     'Autonomously Explore Commanded', 'Arm Commanded', 'Takeoff Commanded',
                     'Offboard Commanded'):
            command = BehaviorTreeCommand()
            command.condition_name = name
            command.status = Status.SUCCESS if name == selected else Status.FAILURE
            msg.commands.append(command)
        self.pub.publish(msg)
        self.sent_at = time.monotonic()
        print('AUTO SEND ONCE: ' + selected, flush=True)

    def start_takeoff(self):
        self.attempts += 1
        self.owns_takeoff = True
        self.last_fcu_warning = ''
        self.send('Auto Takeoff Commanded')
        self.arm_deadline = self.sent_at + self.ARM_TIMEOUT
        print('AUTO TAKEOFF attempt %d/%d; waiting for command ACK, then FCU armed=true' %
              (self.attempts, self.MAX_ATTEMPTS), flush=True)
        self.move('ACK_TAKEOFF')

    def clear_failed_takeoff(self):
        # Only retract our own failed request. Never clear a user's other modes.
        msg = BehaviorTreeCommands()
        command = BehaviorTreeCommand()
        command.condition_name = 'Auto Takeoff Commanded'
        command.status = Status.FAILURE
        msg.commands.append(command)
        self.pub.publish(msg)
        self.sent_at = time.monotonic()
        self.ready_since = None
        print('AUTO arm not confirmed after %.0fs; clearing failed takeoff request (attempt %d/%d); %s%s' %
              (self.ARM_TIMEOUT, self.attempts, self.MAX_ATTEMPTS, self.wait_reason(),
               '; last FCU warning: ' + self.last_fcu_warning if self.last_fcu_warning else ''), flush=True)
        self.move('RESET_TAKEOFF')

    def tick(self):
        if self.phase == 'DONE':
            return
        now = time.monotonic()
        # A user's mode selection takes precedence over pending automation.
        # With --auto-plan alone, manual arm/takeoff is the expected prerequisite.
        manual_modes = self.MANUAL_MODES + (self.MANUAL_START_MODES if self.takeoff else ())
        for key in manual_modes:
            if self.truth(key):
                print('AUTO CANCELLED: manual mode ' + key, flush=True)
                self.move('DONE')
                return
        if self.truth('global_plan_commanded') and self.phase not in ('WAIT_PLAN', 'ACK_PLAN'):
            print('AUTO CANCELLED: global plan already selected externally', flush=True)
            self.move('DONE')
            return
        if (self.owns_takeoff and self.phase in ('WAIT_ARMED', 'WAIT_FCU_SYNC', 'WAIT_TAKEOFF', 'WAIT_PLAN')
                and self.fresh('auto_takeoff_commanded') and not self.values['auto_takeoff_commanded']
                and self.seen['auto_takeoff_commanded'] >= self.sent_at):
            print('AUTO CANCELLED: takeoff request cleared externally', flush=True)
            self.move('DONE')
            return
        if now - self.phase_start > self.timeout:
            self.fail('automatic startup timed out; no further commands will be sent')
        if now - self.last_report >= 10:
            self.last_report = now
            state = self.values.get('state')
            print('AUTO WAIT %s | ready=%s prearm=%s armed=%s landed=%s airborne=%s map=%s path=%s attempt=%d/%d | %s' %
                  (self.phase, self.ready(), self.prearm_ready(),
                   state.armed if state and self.fresh('state') else 'unknown',
                   self.values.get('landed') if self.fresh('landed') else 'unknown',
                   self.airborne(), self.truth('map', 60), self.truth('path', 30),
                   self.attempts, self.MAX_ATTEMPTS, self.wait_reason()), flush=True)
        if self.phase == 'WAIT_READY':
            if self.ever_armed_or_airborne or self.truth('auto_takeoff_commanded'):
                # Observe an existing/manual sequence; never take ownership of it.
                self.move('WAIT_TAKEOFF')
                return
            if not (self.ground_unarmed() and self.prearm_ready()):
                self.ready_since = None
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < self.STABLE_SECONDS:
                return
            self.start_takeoff()
        elif self.phase == 'ACK_TAKEOFF':
            if self.truth('auto_takeoff_commanded') and self.seen['auto_takeoff_commanded'] >= self.sent_at:
                print('AUTO takeoff request acknowledged; this does not confirm arming', flush=True)
                self.move('WAIT_ARMED')
            elif now - self.sent_at > self.ACK_TIMEOUT:
                self.fail('takeoff request not acknowledged; not retrying')
        elif self.phase == 'WAIT_ARMED':
            if self.fresh('state') and self.values['state'].connected and self.values['state'].armed:
                print('AUTO FCU armed=true confirmed; takeoff retries disabled', flush=True)
                self.move('WAIT_TAKEOFF')
            elif self.ever_armed_or_airborne:
                # EXTENDED_SYS_STATE can report takeoff before the slower
                # HEARTBEAT updates State.armed. This locks out retries, but
                # is not by itself a disarm event or a startup failure.
                print('AUTO takeoff/arming evidence received; retries disabled, waiting up to %.0fs '
                      'for FCU state synchronization; %s' %
                      (self.STATE_SYNC_TIMEOUT, self.state_snapshot()), flush=True)
                self.move('WAIT_FCU_SYNC')
            elif now >= self.arm_deadline and self.ground_unarmed():
                self.clear_failed_takeoff()
        elif self.phase == 'WAIT_FCU_SYNC':
            if self.fresh('state') and self.values['state'].connected and self.values['state'].armed:
                print('AUTO FCU state synchronized: armed=true confirmed; ' + self.state_snapshot(), flush=True)
                self.move('WAIT_TAKEOFF')
            elif now - self.phase_start >= self.STATE_SYNC_TIMEOUT:
                self.fail('FCU state synchronization timed out after takeoff/arming evidence; not retrying')
        elif self.phase == 'RESET_TAKEOFF':
            if self.ever_armed_or_airborne:
                self.fail('FCU armed/airborne observed during retry reset; not retrying')
            actions = {a.strip() for a in self.values.get('actions', '').split(',')}
            if (self.ground_unarmed() and self.fresh('auto_takeoff_commanded')
                    and not self.values['auto_takeoff_commanded']
                    and self.seen['auto_takeoff_commanded'] >= self.sent_at
                    and self.fresh('actions') and self.seen['actions'] >= self.sent_at
                    and not actions.intersection(('Arm', 'Takeoff', 'Request Control'))):
                if self.attempts >= self.MAX_ATTEMPTS:
                    self.fail('arming failed: maximum takeoff attempts reached')
                self.retry_at = now + self.RETRY_DELAY
                self.move('RETRY_WAIT')
            elif now - self.sent_at > self.ACK_TIMEOUT:
                self.fail('failed takeoff request/Arm action did not deactivate; not retrying')
        elif self.phase == 'RETRY_WAIT':
            if self.ever_armed_or_airborne:
                print('AUTO CANCELLED: FCU armed/airborne during retry wait', flush=True)
                self.move('DONE')
                return
            if self.truth('auto_takeoff_commanded'):
                print('AUTO CANCELLED: takeoff request selected externally during retry wait', flush=True)
                self.move('DONE')
                return
            if not (self.ground_unarmed() and self.prearm_ready()
                    and self.fresh('auto_takeoff_commanded')
                    and not self.values['auto_takeoff_commanded']):
                self.ready_since = None
                return
            if self.ready_since is None:
                self.ready_since = now
            if now >= self.retry_at and now - self.ready_since >= self.STABLE_SECONDS:
                self.start_takeoff()
        elif self.phase == 'WAIT_TAKEOFF':
            if self.airborne():
                print('AUTO takeoff complete confirmed (armed + OFFBOARD + IN_AIR + takeoff_complete)', flush=True)
                self.move('WAIT_PLAN' if self.plan else 'DONE')
        elif self.phase == 'WAIT_PLAN':
            if self.airborne() and self.truth('map', 60) and self.truth('path', 30):
                if self.truth('global_plan_commanded') and self.truth('actions') and 'Follow Global Plan' in self.values['actions']:
                    self.move('DONE')
                else:
                    self.send('Global Plan Commanded')
                    self.move('ACK_PLAN')
        elif self.phase == 'ACK_PLAN':
            if (self.truth('global_plan_commanded') and self.seen['global_plan_commanded'] >= self.sent_at
                    and self.truth('actions') and self.seen['actions'] >= self.sent_at
                    and 'Follow Global Plan' in self.values['actions']):
                print('AUTO following global plan confirmed; automatic startup finished', flush=True)
                self.move('DONE')
            elif now - self.sent_at > self.ACK_TIMEOUT:
                self.fail('Global Plan not confirmed; not retrying')

def main():
    rclpy.init()
    node = rclpy.create_node('raven_console_auto_start', enable_rosout=False)
    controller = AutoStart(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print('AUTO_FAILURE ' + json.dumps({'phase': controller.phase, 'attempts': controller.attempts,
                                           'reason': str(exc)}, ensure_ascii=False), flush=True)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


# Read-only telemetry: no goal positions or evaluator decisions reach the planner.
EPISODE_HELPER = r'''
import json, time, rclpy
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('raven_console_evaluation', enable_rosout=False,
                        start_parameter_services=False)
seen, values = {}, {}
last_clock = None
def record(key, value):
    values[key] = value
    seen[key] = time.monotonic()
def clock_cb(msg):
    global last_clock
    value = msg.clock.sec + msg.clock.nanosec / 1e9
    if value != last_clock:
        record('clock', value)
    last_clock = value
def odom_cb(msg):
    p = msg.pose.pose.position
    record('odom', {'position': [p.x, p.y, p.z], 'frame': msg.header.frame_id,
                   'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9})
def state_cb(msg):
    record('state', {'armed': msg.armed, 'connected': msg.connected, 'mode': msg.mode})
def bool_cb(key):
    def callback(msg): record(key, msg.data)
    return callback
node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data)
node.create_subscription(Odometry, '/robot_1/odometry_conversion/odometry', odom_cb, qos_profile_sensor_data)
node.create_subscription(State, '/robot_1/interface/mavros/state', state_cb, qos_profile_sensor_data)
node.create_subscription(PointCloud2, '/rayfronts/voxel_rgb',
    lambda msg: record('map', bool(msg.width * msg.height)), qos_profile_sensor_data)
for key in ('global_plan_commanded', 'takeoff_complete', 'pause_commanded', 'land_commanded',
            'disarm_commanded', 'rewind_commanded', 'state_estimate_timed_out'):
    node.create_subscription(Bool, '/robot_1/behavior/' + key + '_success', bool_cb(key), 10)
node.create_subscription(String, '/robot_1/behavior/active_actions',
    lambda msg: record('actions', msg.data), 10)
lvlm_sequence = 0
def lvlm_cb(msg):
    global lvlm_sequence
    lvlm_sequence += 1
    record('lvlm_output', {'text': msg.data, 'sequence': lvlm_sequence})
node.create_subscription(String, '/lvlm_output', lvlm_cb, 10)
def report():
    now = time.monotonic()
    print('RAVEN_EVENT ' + json.dumps({'values': values,
          'ages': {k: now - v for k, v in seen.items()}}, allow_nan=False), flush=True)
node.create_timer(0.2, report)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
'''


def load_goals(scene, task, pose):
    path = ROOT / 'RayFronts/rayfronts/annotations' / (scene + '.json')
    data = json.loads(path.read_text())
    classes = set(x.strip().lower() for x in task.split(',') if x.strip())
    available = set(x['class'].strip().lower() for x in data)
    missing = classes - available
    if missing:
        raise ValueError('标注中没有类别 ' + ', '.join(sorted(missing)) +
                         '；可选：' + ', '.join(sorted(available)))
    offset = [float(pose['DRONE_' + k]) for k in ('X', 'Y', 'Z')]
    goals = []
    for index, item in enumerate(data):
        if item['class'].strip().lower() not in classes:
            continue
        box = item['bbox_world']
        center, size = box['center_xyz_m'], box['size_xyz_m']
        angles = box.get('orientation_deg', [0., 0., 0.])
        if (any(len(x) != 3 for x in (center, size, angles)) or
                not all(math.isfinite(v) for v in [*center, *size, *angles]) or
                any(v <= 0 for v in size)):
            raise ValueError('无效标注包围盒：' + str(index))
        # Same ENU translation as upstream annotation_viz.py. MAVROS local ENU
        # axes are not rotated by the vehicle's initial heading.
        goals.append({'id': index, 'class': item['class'],
                      'center': [c - o for c, o in zip(center, offset)],
                      'size': size, 'orientation_deg': angles})
    if not goals:
        raise ValueError('任务没有可评分的真实目标')
    return goals, str(path), hashlib.sha256(path.read_bytes()).hexdigest()


def box_distance(point, goal):
    """Euclidean point-to-oriented-box distance, metres; zero inside the box."""
    x, y, z = [p - c for p, c in zip(point, goal['center'])]
    roll, pitch, yaw = map(math.radians, goal['orientation_deg'])
    # Apply inverse Rz(yaw) Ry(pitch) Rx(roll).
    x, y = math.cos(yaw)*x + math.sin(yaw)*y, -math.sin(yaw)*x + math.cos(yaw)*y
    x, z = math.cos(pitch)*x - math.sin(pitch)*z, math.sin(pitch)*x + math.cos(pitch)*z
    y, z = math.cos(roll)*y + math.sin(roll)*z, -math.sin(roll)*y + math.cos(roll)*z
    return math.sqrt(sum(max(abs(v) - s/2, 0.)**2 for v, s in zip((x, y, z), goal['size'])))


class Episode:
    """Host-side scoring only. All local protocol choices are recorded explicitly."""
    def __init__(self, args, run_dir, emit, now=time.monotonic):
        self.args, self.run_dir, self.emit, self.now = args, run_dir, emit, now
        self.goals, annotation, digest = load_goals(args.env, args.task, args.pose)
        self.created = now()
        self.last_event = self.created
        self.started = None
        self.start_sim = None
        self.last_sim = None
        self.last_stamp = None
        self.last_position = None
        self.last_report = 0.
        self.hits = {}
        self.distance = 0.
        self.done = False
        self.telemetry = None
        self.trajectory = None
        self.result = {'schema_version': 1, 'status': 'waiting_for_navigation',
            'scene': args.env, 'task': args.task, 'pose': args.pose,
            'annotation_file': annotation, 'annotation_sha256': digest,
            'protocol': 'local_progress_validation', 'paper_exact_reproduction': False,
            'paper': 'https://arxiv.org/html/2509.23563v1#S4.SS3',
            'protocol_note': 'Progress K/M follows paper; distance threshold, budget and clock origin are local choices; author values not published in reviewed sources.',
            'goal_radius_m': args.goal_radius, 'waypoint_radius_m': args.waypoint_radius,
            'episode_seconds': args.episode_seconds,
            'budget_clock': 'simulation /clock', 'start_condition': 'armed OFFBOARD, takeoff complete, Follow Global Plan confirmed',
            'pose_source': '/robot_1/odometry_conversion/odometry (MAVROS estimate)',
            'annotation_transform': 'world ENU minus configured spawn translation; same as annotation_viz.py',
            'arrival_rule': 'sampled 3D distance to oriented ground-truth box <= radius; each instance once',
            'collision_detection': 'not available; proximity score is not collision-free certification',
            'PPL': None, 'PPL_note': 'Oracle optimal path d_K is unavailable',
            'lvlm_enabled': args.lvlm, 'macvo_enabled': not args.disable_macvo,
            'random_walk_enabled': args.keep_random_walk, 'goals': self.goals,
            'target_count': len(self.goals), 'reached_count': 0, 'progress': 0.,
            'elapsed_sim_s': 0., 'elapsed_wall_s': 0., 'path_length_m': 0.,
            'path_to_last_goal_m': None, 'reached': [], 'lvlm_responses': 0}
        self.last_lvlm = None
        self.save()
        emit('EPISODE', f'真实目标={len(self.goals)}；半径={args.goal_radius}m；预算={args.episode_seconds}s 仿真时间。参数来源=本地验证，非论文原值。')

    def save(self):
        self.result.update(reached_count=len(self.hits), progress=len(self.hits)/len(self.goals),
                           path_length_m=self.distance, reached=list(self.hits.values()))
        temporary = self.run_dir / 'result.json.tmp'
        temporary.write_text(json.dumps(self.result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.run_dir / 'result.json')

    def finish(self, reason, detail=''):
        if self.done:
            return
        self.done = True
        self.result.update(status=reason, detail=detail,
                           ended_at=datetime.datetime.now().astimezone().isoformat())
        if self.started is not None:
            self.result['elapsed_wall_s'] = self.now() - self.started
        self.save()
        if self.trajectory:
            self.trajectory.close()
        self.emit('RESULT', f'{reason} | Progress={len(self.hits)}/{len(self.goals)} '
                  f'({100*len(self.hits)/len(self.goals):.2f}%) | {detail} | {self.run_dir / "result.json"}')

    def event(self, event):
        if self.done:
            return
        now = self.now()
        self.last_event, self.telemetry = now, event
        values, ages = event['values'], event['ages']
        def fresh(key, limit=5.):
            return key in values and 0 <= ages.get(key, math.inf) <= limit
        if fresh('lvlm_output'):
            response = values['lvlm_output']
            # Count response arrivals (even repeated text), not 5 Hz telemetry copies.
            token = response['sequence']
            if token != self.last_lvlm:
                self.last_lvlm = token
                self.result['lvlm_responses'] += 1
                self.emit('LVLM-RESULT', response['text'])
        if not fresh('clock') or not fresh('odom'):
            return
        sim = values['clock']
        odom = values['odom']
        point, stamp = odom['position'], odom['stamp']
        if (odom['frame'] != 'map' or len(point) != 3 or
                not all(math.isfinite(v) for v in [sim, stamp, *point])):
            self.finish('invalid_pose', '需要有限数值的 map 坐标位置')
            return
        state = values.get('state', {})
        if self.started is None:
            ready = (fresh('state') and state.get('connected') and state.get('armed') and
                state.get('mode') == 'OFFBOARD' and fresh('takeoff_complete') and values['takeoff_complete'] and
                fresh('global_plan_commanded') and values['global_plan_commanded'] and
                fresh('actions') and 'Follow Global Plan' in values['actions'] and
                fresh('map', 60.) and values['map'])
            if not ready:
                return
            self.started, self.start_sim = now, sim
            self.result.update(status='running', started_at=datetime.datetime.now().astimezone().isoformat())
            self.trajectory = (self.run_dir / 'trajectory.jsonl').open('w', buffering=1)
            self.emit('EPISODE', 'START：导航已确认，开始计算预算和 Progress。')
        if self.last_sim is not None and sim < self.last_sim - .001:
            self.finish('clock_reset', '仿真时钟倒退，不能合并不同轮次')
            return
        self.last_sim = sim
        elapsed = sim - self.start_sim
        self.result.update(elapsed_sim_s=elapsed, elapsed_wall_s=now-self.started)
        if elapsed > self.args.episode_seconds:
            self.finish('budget_exhausted', '仿真时间预算耗尽，保留部分完成比例')
            return
        if fresh('state') and (not state.get('connected') or not state.get('armed') or state.get('mode') != 'OFFBOARD'):
            self.finish('flight_state_changed', '飞行连接/解锁/OFFBOARD 状态变化')
            return
        for key in ('pause_commanded', 'land_commanded', 'disarm_commanded', 'rewind_commanded'):
            if fresh(key) and values[key]:
                self.finish('mode_changed', key)
                return
        if stamp != self.last_stamp:
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.finish('odometry_reset', '里程计时间倒退')
                return
            if self.last_position is not None:
                self.distance += math.dist(point, self.last_position)
            self.last_position, self.last_stamp = point, stamp
            self.trajectory.write(json.dumps({'sim_elapsed_s': elapsed, 'wall_elapsed_s': now-self.started,
                'odom_stamp': stamp, 'position_map': point, 'path_length_m': self.distance})+'\n')
            for goal in self.goals:
                if goal['id'] in self.hits:
                    continue
                distance = box_distance(point, goal)
                if distance <= self.args.goal_radius:
                    self.hits[goal['id']] = {'id': goal['id'], 'class': goal['class'],
                        'distance_m': distance, 'position_map': point, 'elapsed_sim_s': elapsed,
                        'elapsed_wall_s': now-self.started, 'path_length_m': self.distance}
                    self.result['path_to_last_goal_m'] = self.distance
                    self.emit('EPISODE', f'REACHED {goal["class"]}#{goal["id"]} distance={distance:.3f}m；{len(self.hits)}/{len(self.goals)}')
                    self.save()
            if len(self.hits) == len(self.goals):
                self.finish('all_goals_reached', '全部真实目标实例满足配置的距离条件')
                return
        if elapsed >= self.args.episode_seconds:
            self.finish('budget_exhausted')

    def tick(self):
        if self.done:
            return
        now = self.now()
        if self.started is None:
            if now-self.created > self.args.auto_timeout:
                self.finish('startup_timeout', '在启动等待上限内未进入导航')
        else:
            if now-self.started > self.args.episode_wall_timeout:
                self.finish('wall_watchdog', '真实时间保护上限；不算论文预算正常结束')
            event = self.telemetry or {'ages': {}}
            for key in ('clock', 'odom', 'map', 'state', 'actions'):
                age = event['ages'].get(key, math.inf) + now-self.last_event
                if age > self.args.data_timeout:
                    self.finish('data_stale', f'{key} 未更新 {age:.1f}s')
                    break
        if not self.done and now-self.last_report >= 10:
            self.last_report = now
            self.save()
            self.emit('EPISODE', f'{self.result["status"]} | Progress={len(self.hits)}/{len(self.goals)} '
                f'| sim={self.result["elapsed_sim_s"]:.1f}/{self.args.episode_seconds:.1f}s '
                f'| wall={self.result["elapsed_wall_s"]:.1f}s | path={self.distance:.1f}m')


def arguments(argv=None):
    scenes = sorted(p.name.removesuffix('_Launch.py') for p in
                    (STACK / 'simulation/isaac-sim/launch_scripts').glob('*_Launch.py'))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--env', choices=scenes, default='RetroNeighborhood', help='场景名')
    parser.add_argument('--task', help='语义目标，例如 "water tower"；未填时在启动前输入')
    parser.add_argument('--lvlm', action='store_true', help='同时开启可选 LVLM（额外占用显存）')
    parser.add_argument('--disable-macvo', action='store_true', help='不启动当前仿真控制链路未使用的 MACVO')
    parser.add_argument('--keep-random-walk', action='store_true', help='恢复 AirStack 随机规划器（会与 RayFronts 共用全局路径话题）')
    parser.add_argument('--episode-seconds', type=float, help='启用评估：导航开始后的仿真时间预算；必须同时指定 --goal-radius')
    parser.add_argument('--goal-radius', type=float, help='到真实目标包围盒的距离阈值（米）；作者原始评估阈值未公开')
    parser.add_argument('--waypoint-radius', type=float, default=5., help='体素导航访问候选的航点距离阈值（米），默认 5；原导航值为 3；独立于 --goal-radius')
    parser.add_argument('--episode-wall-timeout', type=float, default=3600, help='评估的真实时间保护上限，默认 3600 秒')
    parser.add_argument('--data-timeout', type=float, default=60, help='评估中关键数据持续中断的真实秒数，默认 60')
    parser.add_argument('--min-free-vram', type=float, default=512, help='评估时空闲显存连续 5 秒低于该 MiB 值则记录资源不足并结束；默认 512，0 关闭；不能保证捕获瞬时 OOM')
    parser.add_argument('--low-vram', action='store_true',
                        help='关闭 Isaac 观察窗口/视口刷新，RayFronts 使用 expandable_segments；不改算法或传感器')
    parser.add_argument('--auto-takeoff', action='store_true', help='就绪后自动执行 Arm and Takeoff')
    parser.add_argument('--auto-plan', action='store_true', help='起飞完成且地图/路径更新后自动执行 Global Plan；不隐含自动起飞')
    parser.add_argument('--auto-timeout', type=float, default=900,
                        help='自动流程每个等待阶段的超时秒数，默认 900；指令确认超时为 15 秒')
    for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
        parser.add_argument('--' + key, type=float, help='覆盖出生位置/四元数')
    parser.add_argument('--restart', action='store_true', help='停止已有 RAVEN 容器，重新启动仿真')
    parser.add_argument('--dry-run', action='store_true', help='只显示计划，不访问 Docker、不启动服务')
    parser.add_argument('--skip-build', action='store_true', help='跳过原启动流程的 colcon build')
    parser.add_argument('--log-dir', type=Path, help='日志目录；默认 ~/.cache/raven/console/<时间>')
    args = parser.parse_args(argv)
    if not math.isfinite(args.auto_timeout) or args.auto_timeout <= 0:
        parser.error('--auto-timeout 必须为有限正数')
    if (args.episode_seconds is None) != (args.goal_radius is None):
        parser.error('--episode-seconds 和 --goal-radius 必须一起指定；不隐含论文原值')
    for key in ('episode_seconds', 'goal_radius', 'waypoint_radius', 'episode_wall_timeout', 'data_timeout'):
        value = getattr(args, key)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error('--' + key.replace('_', '-') + ' 必须为有限正数')
    if not math.isfinite(args.min_free_vram) or args.min_free_vram < 0:
        parser.error('--min-free-vram 必须是有限非负数')
    if args.task is None:
        if args.dry_run:
            args.task = '<启动时输入任务>'
        elif sys.stdin.isatty():
            args.task = input('请输入语义目标（例如 water tower）：').strip()
        else:
            parser.error('非交互终端必须提供 --task')
    if not args.task.strip():
        parser.error('--task 不能为空')
    pose = [0., 0., .07, 0., 0., 0., 1.]
    defaults = {'FireAcademy': [25.7, 11.5, .07, 0., 0., -.93, .366],
                'ConstructionSite': [-30., -1.5, .07, 0., 0., 0., 1.],
                'AbandonedFactory': [7.57, -5.5, .71, 0., 0., 0., 1.]}
    pose = defaults.get(args.env, pose)
    keys = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    pose = [getattr(args, k) if getattr(args, k) is not None else v for k, v in zip(keys, pose)]
    if not all(math.isfinite(v) for v in pose) or sum(v*v for v in pose[3:]) < 1e-12:
        parser.error('位姿必须为有限数字，四元数不能全为零')
    args.pose = dict(zip(('DRONE_' + k.upper() for k in keys), map(str, pose)))
    if args.episode_seconds is not None:
        try:
            load_goals(args.env, args.task, args.pose)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.error('无法建立评估任务：' + str(exc))
    return args


class Console:
    def __init__(self, args):
        self.args = args
        self.env = os.environ.copy()
        self.children = {}
        self.events = queue.Queue()
        self.failed = set()
        self.auto_failure = None
        self.containers = []
        self.stopping = False
        self.last_status = 0.
        self.handlers = {}
        self.repeated_notices = {}
        self.lvlm_ready = False
        self.low_memory_since = None
        self.run_dir = args.log_dir or (Path.home() / '.cache/raven/console' /
                                       datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.episode = Episode(args, self.run_dir, self.emit) if args.episode_seconds is not None else None

    def emit(self, label, message):
        message = ANSI.sub('', message).rstrip('\r\n')
        line = f'[{datetime.datetime.now():%H:%M:%S}] [{label}] {message}'
        for name in ('all', label):
            if name not in self.handlers:
                handler = RotatingFileHandler(self.run_dir / (name + '.log'),
                                              maxBytes=64*1024*1024, backupCount=3, encoding='utf-8')
                self.handlers[name] = handler
            self.handlers[name].emit(logging.LogRecord(name, logging.INFO, '', 0, line, (), None))
        # Keep complete raw logs, but don't flood the terminal with one notice
        # emitted by bitsandbytes for every quantized matrix multiplication.
        if label == 'LVLM' and 'MatMul8bitLt: inputs will be cast from torch.bfloat16 to float16' in message:
            now = time.monotonic()
            count, last = self.repeated_notices.get(label, (0, -math.inf))
            self.repeated_notices[label] = (count + 1, last)
            if count and now-last < 10:
                return
            self.repeated_notices[label] = (count + 1, now)
            line += f' [同类提示累计 {count+1} 次；原文全部保存在 LVLM.log]'
        print(line, flush=True)

    def spawn(self, label, cmd):
        proc = subprocess.Popen(cmd, cwd=STACK, env=self.env, stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors='replace', bufsize=1, start_new_session=True)
        self.children[label] = proc
        def read():
            for line in proc.stdout:
                self.events.put((label, line))
            proc.stdout.close()
            self.events.put((label, None))
        threading.Thread(target=read, daemon=True).start()
        return proc

    def pump(self, timeout=.2):
        try:
            label, line = self.events.get(timeout=timeout)
        except queue.Empty:
            return
        if line is not None:
            if label == 'EVALUATION' and line.startswith('RAVEN_EVENT '):
                if self.episode and not self.stopping:
                    try:
                        self.episode.event(json.loads(line[len('RAVEN_EVENT '):]))
                    except Exception as exc:
                        self.episode.finish('evaluation_error', str(exc))
                return
            self.emit(label, line)
            if label == 'AUTO' and line.startswith('AUTO_FAILURE ') and not self.stopping:
                try:
                    self.auto_failure = json.loads(line[len('AUTO_FAILURE '):])
                except (ValueError, TypeError):
                    self.auto_failure = {'reason': line.strip()}
                self.emit('ALERT', '自动启动失败，将结束本轮：' + str(self.auto_failure))
                if self.episode:
                    self.episode.result['auto_start_failure'] = self.auto_failure
                    self.episode.finish('auto_start_failed', str(self.auto_failure))
            if label == 'LVLM' and 'InternVL3-2B model loaded!' in line:
                self.lvlm_ready = True
            if label == 'GPU' and self.episode and not self.stopping:
                columns = line.strip().split(',')
                if len(columns) == 4:
                    try:
                        used, free = float(columns[1]), float(columns[2])
                        self.episode.result['gpu_peak_used_mib'] = max(used, self.episode.result.get('gpu_peak_used_mib', 0))
                        if self.args.min_free_vram and free < self.args.min_free_vram:
                            if self.low_memory_since is None:
                                self.low_memory_since = time.monotonic()
                                self.emit('ALERT', f'显存保护观察：free={free:.0f}MiB')
                            elif time.monotonic()-self.low_memory_since >= 5:
                                self.episode.finish('resource_limit', f'空闲显存持续低于 {self.args.min_free_vram}MiB')
                        else:
                            self.low_memory_since = None
                    except ValueError:
                        pass
            if 'CUDA out of memory' in line:
                self.emit('ALERT', f'!!! {label} 显存不足；请查看上方堆栈和 {label}.log !!!')
            return
        code = self.children[label].wait()
        if not self.stopping and label not in ('SETUP', 'STOP'):
            self.failed.add(label)
            self.emit('ALERT', f'!!! {label} 已退出，exit={code}；容器 Up 不代表服务正常 !!!')
            if self.episode and label in ('ISAAC', 'ROBOT', 'RAYFRONTS', 'TASK', 'AUTO', 'LVLM', 'EVALUATION', 'GPU'):
                self.episode.finish('process_failed', f'{label} exit={code}')

    def run(self, cmd, timeout=30, check=True):
        result = subprocess.run(cmd, cwd=STACK, env=self.env, capture_output=True,
                                text=True, timeout=timeout)
        if check and result.returncode:
            raise RuntimeError(f'{shlex.join(cmd[:3])}: {result.stderr.strip() or result.stdout.strip()}')
        return result

    def inspect(self, name):
        result = self.run(['docker', 'inspect', name], check=False)
        return json.loads(result.stdout)[0] if result.returncode == 0 else None

    def exec_cmd(self, container, command, extra=None, workdir=None):
        cmd = ['docker', 'exec', '-e', 'PYTHONUNBUFFERED=1', '-e', 'RCUTILS_LOGGING_BUFFERED_STREAM=0']
        values = dict(self.args.pose, ISAAC_SIM_SCRIPT_NAME=self.args.env + '_Launch.py',
                      ROS_DOMAIN_ID=self.env.get('ROS_DOMAIN_ID', '1'))
        values.update(extra or {})
        for key, value in values.items():
            cmd += ['-e', key + '=' + value]
        if workdir:
            cmd += ['-w', workdir]
        # Some images return early from .bashrc in noninteractive shells. -i loads
        # the same ROS setup as the original launcher; docker exec needs no TTY.
        return cmd + [container, 'bash', '-ic', command]

    def setup(self):
        # Load trusted local .env, then override only this run; never rewrite it.
        result = self.run(['bash', '-c', 'set -a; source "$1" && env -0', 'raven', str(STACK / '.env')])
        self.env.update(dict(entry.split('=', 1) for entry in result.stdout.split('\0') if '=' in entry))
        self.env.update(self.args.pose)
        self.env.update(ISAAC_SIM_SCRIPT_NAME=self.args.env + '_Launch.py', AUTOLAUNCH='false',
                        ISAAC_SIM_USE_STANDALONE='true', PLAY_SIM_ON_START='true', NUM_ROBOTS='1')
        self.run(['docker', 'info', '--format', '{{.DockerRootDir}}'])
        project = self.env.get('COMPOSE_PROJECT_NAME', 'airstack')
        # Discover robot container from Compose labels; don't assume its generated name.
        robots = self.run(['docker', 'ps', '-a', '--filter', 'label=com.docker.compose.project=' + project,
                          '--filter', 'label=com.docker.compose.service=robot', '--format', '{{.Names}}']).stdout.split()
        if len(robots) > 1:
            raise RuntimeError('检测到多个 robot 容器，此启动器仅支持单机单无人机。')
        existing = ['isaac-sim', *robots, RAY, LVLM]
        infos = {name: self.inspect(name) for name in existing}
        for name, destination, source in [(RAY, '/workspace/RayFronts', ROOT / 'RayFronts'),
                                           (LVLM, '/app', ROOT / 'LVLM')]:
            info = infos[name]
            if info and not any(m['Destination'] == destination and
                                Path(m['Source']).resolve() == source.resolve() for m in info['Mounts']):
                raise RuntimeError(f'{name} 属于另一目录，拒绝接管。')
            if info:
                command = ' '.join((info['Config'].get('Entrypoint') or []) +
                                   (info['Config'].get('Cmd') or []))
                if any(token in command for token in ('mapping_server', 'internvl3', 'input_prompt', 'annotation_viz')):
                    raise RuntimeError(f'{name} 自带自动启动命令，无法重复启动。请先停止并移除此旧容器后重试（保留镜像/缓存）。')
        running = [name for name, info in infos.items() if info and info['State']['Running']]
        if running and not self.args.restart:
            raise RuntimeError('已有服务运行：' + ', '.join(running) +
                               '。要重新开始请加 --restart（会结束当前仿真）。')
        if running:
            self.emit('SETUP', '停止旧运行：' + ', '.join(running))
            self.run(['docker', 'stop', '-t', '15', *running], timeout=90)
        # Track the expected names before startup so partial startup is also cleaned up.
        self.containers = ['isaac-sim', *(robots or [project + '-robot-1'])]
        proc = self.spawn('SETUP', ['bash', str(STACK / 'airstack.sh'), 'up',
                                  '--force-recreate', '--no-build', 'isaac-sim', 'robot'])
        while proc.poll() is None:
            self.pump()
        while not self.events.empty():
            self.pump(0)
        if proc.returncode:
            raise RuntimeError('AirStack 容器启动失败，见 SETUP.log')
        robots = self.run(['docker', 'ps', '--filter', 'label=com.docker.compose.project=' + project,
                          '--filter', 'label=com.docker.compose.service=robot', '--format', '{{.Names}}']).stdout.split()
        if len(robots) != 1:
            raise RuntimeError('无法唯一确定已启动的 robot 容器')
        robot = robots[0]
        self.containers = ['isaac-sim', robot]
        self.start_aux(RAY, 'rayfronts:desktop', ROOT / 'RayFronts', '/workspace/RayFronts')
        if self.args.lvlm:
            self.start_aux(LVLM, 'lvlm:latest', ROOT / 'LVLM', '/app')
            # Cache lives on /home. Transformers must not use the image's old /app/cache path.
            self.spawn('LVLM', self.exec_cmd(LVLM, 'exec python3 -u internvl3.py',
                {'HF_HOME': '/root/.cache/huggingface', 'TRANSFORMERS_CACHE': '/root/.cache/huggingface/hub',
                 'PYTORCH_CUDA_ALLOC_CONF': 'expandable_segments:True'}, workdir='/app'))
            deadline = time.monotonic() + self.args.auto_timeout
            while not self.lvlm_ready:
                self.pump()
                if self.children['LVLM'].poll() is not None:
                    raise RuntimeError('LVLM 未成功加载，终止启动，见 LVLM.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('LVLM 加载超时，终止启动')
            self.emit('SETUP', 'LVLM 模型加载已确认；继续启动仿真与导航。')
        for name in self.containers:
            self.spawn('DOCKER-' + name, ['docker', 'logs', '--follow', '--since', '1m', name])
        scene = '/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/' + self.args.env + '_Launch.py'
        self.spawn('ISAAC', self.exec_cmd('isaac-sim',
                   'run_isaac_python ' + shlex.quote(scene) +
                   ' --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts',
                   {'PLAY_SIM_ON_START': 'true', 'RAVEN_HEADLESS': '1' if self.args.low_vram else '0'}))
        launch = shlex.join(['ros2', 'launch', self.env.get('ROBOT_LAUNCH_PACKAGE', 'robot_bringup'),
                             self.env.get('ROBOT_LAUNCH_FILE', 'robot.launch.xml')])
        self.spawn('ROBOT', self.exec_cmd(robot,
                   ('' if self.args.skip_build else 'bws && ') + 'sws && exec ' + launch,
                   {'RAVEN_ENABLE_MACVO': 'false' if self.args.disable_macvo else 'true',
                    'RAVEN_ENABLE_RANDOM_WALK': 'true' if self.args.keep_random_walk else 'false'}))
        self.emit('SETUP', f'MACVO={not self.args.disable_macvo}，随机规划器={self.args.keep_random_walk}；RAVEN 原有前沿探索保留。')
        ray_env = {'RAVEN_WAYPOINT_RADIUS': str(self.args.waypoint_radius)}
        if self.args.low_vram:
            ray_env['PYTORCH_CUDA_ALLOC_CONF'] = 'expandable_segments:True'
        self.emit('SETUP', f'体素导航航点到达阈值 <{self.args.waypoint_radius:g}m；独立于 --goal-radius 评分半径。')
        self.spawn('RAYFRONTS', self.exec_cmd(RAY, 'exec bash /workspace/RayFronts/run_mapping_server_rosnode.sh',
                                             ray_env,
                                             workdir='/workspace/RayFronts'))
        if self.args.low_vram:
            self.emit('SETUP', 'LOW_VRAM: Isaac 无观察窗口、关闭视口刷新；RayFronts expandable_segments=True；此开关不调整传感器配置。')
        if self.args.low_vram or self.episode:
            self.emit('GPU', '列顺序：采样时间, 已用显存(MiB), 空闲显存(MiB), GPU利用率(%)')
            self.spawn('GPU', ['nvidia-smi', '--query-gpu=timestamp,memory.used,memory.free,utilization.gpu',
                               '--format=csv,noheader,nounits', '-l', '2'])
        self.spawn('ANNOTATION', self.exec_cmd(RAY, 'exec python3 -u annotation_viz.py',
                                              workdir='/workspace/RayFronts'))
        self.spawn('TASK', self.exec_cmd(RAY, 'exec python3 -u -c ' + shlex.quote(ROS_HELPER),
                                        {'RAVEN_TASK': self.args.task}, '/workspace/RayFronts'))
        if self.episode:
            self.spawn('EVALUATION', self.exec_cmd(robot, 'sws && exec python3 -u -c ' + shlex.quote(EPISODE_HELPER)))
        if self.args.auto_takeoff or self.args.auto_plan:
            self.spawn('AUTO', self.exec_cmd(robot, 'sws && exec python3 -u -c ' + shlex.quote(AUTO_HELPER),
                       {'RAVEN_AUTO_TAKEOFF': str(int(self.args.auto_takeoff)),
                        'RAVEN_AUTO_PLAN': str(int(self.args.auto_plan)),
                        'RAVEN_AUTO_TIMEOUT': str(self.args.auto_timeout)}))
        self.emit('STATUS', '所有启动命令已发出，等待模型/场景加载及 DATA 心跳；'
                  f'自动起飞={self.args.auto_takeoff}，自动跟随全局规划={self.args.auto_plan}。')

    def start_aux(self, name, image, host_dir, container_dir):
        info = self.inspect(name)
        self.containers.append(name)
        if info:
            # Reuse old containers to preserve their model cache; explicit exec env
            # above avoids stale scene/pose from the old container's configuration.
            self.run(['docker', 'start', name])
            return
        cache = Path.home() / '.cache/raven'
        cmd = ['docker', 'run', '-d', '--name', name, '--gpus', 'all', '--network', 'host',
               '--ipc', 'host', '--privileged', '--runtime=nvidia',
               '-e', 'NVIDIA_DRIVER_CAPABILITIES=all', '-e', 'ROS_DOMAIN_ID=1',
               '-v', f'{host_dir}:{container_dir}', '-w', container_dir]
        caches = ([(cache / 'torch', '/root/.cache/torch'),
                   (cache / 'huggingface', '/root/.cache/huggingface')] if name == RAY else
                  [(Path.home() / '.cache/huggingface', '/root/.cache/huggingface')])
        for host, target in caches:
            host.mkdir(parents=True, exist_ok=True)
            cmd += ['-v', f'{host}:{target}']
        self.run(cmd + ['--entrypoint', '/bin/bash', image, '-c', 'exec sleep infinity'], timeout=60)

    def monitor(self):
        while True:
            self.pump()
            if self.auto_failure is not None:
                return 1
            if self.episode:
                self.episode.tick()
                if self.episode.done:
                    return 0 if self.episode.result['status'] in ('all_goals_reached', 'budget_exhausted') else 1
            if time.monotonic() - self.last_status >= 10:
                self.last_status = time.monotonic()
                labels = [k for k in self.children if k not in ('SETUP', 'STOP') and not k.startswith('DOCKER-')]
                states = [k + ('=运行中' if self.children[k].poll() is None else '=已退出!') for k in labels]
                self.emit('STATUS', ' | '.join(states))
                if self.failed:
                    self.emit('ALERT', '!!! 故障仍未恢复: ' + ', '.join(sorted(self.failed)) +
                              '；未自动重启/悬停，其他程序可能仍继续运行。Ctrl+C 停止整套仿真。')

    def shutdown(self):
        self.stopping = True
        if self.episode and not self.episode.done:
            self.episode.finish('interrupted', '启动器结束或人工中断')
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Cancel in-flight Compose first, preventing containers appearing after stop.
        setup = self.children.get('SETUP')
        if setup and setup.poll() is None:
            os.killpg(setup.pid, signal.SIGTERM)
            try:
                setup.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(setup.pid, signal.SIGKILL)
        if self.containers:
            self.emit('STOP', '停止本次容器（保留镜像/数据）：' + ', '.join(self.containers))
            def stop(name):
                try:
                    result = self.run(['docker', 'stop', '-t', '15', name], timeout=30, check=False)
                    return name, result.returncode, result.stderr.strip()
                except Exception as exc:
                    return name, 1, str(exc)
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                futures = [pool.submit(stop, name) for name in self.containers]
                while any(not f.done() for f in futures):
                    self.pump()
                for future in futures:
                    name, code, error = future.result()
                    self.emit('STOP', name + (' 已停止' if code == 0 else ' 停止失败: ' + error))
        for proc in self.children.values():
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in self.children.values():
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        while not self.events.empty():
            self.pump(0)
        self.emit('STOP', '日志：' + str(self.run_dir))
        for handler in self.handlers.values():
            handler.close()


def main():
    args = arguments()
    if args.dry_run:
        print(json.dumps({'scene': args.env, 'task': args.task, 'pose': args.pose,
                          'services': ['Isaac Sim', 'Robot (含 RViz)', 'RayFronts', 'Annotation',
                                       'Task publisher + data heartbeat'] + (['LVLM'] if args.lvlm else []),
                          'restart_existing': args.restart, 'build_robot_workspace': not args.skip_build,
                          'logs': str(args.log_dir or Path.home() / '.cache/raven/console/<时间>'),
                          'ctrl_c': '停止本次启动的容器', 'auto_arm_takeoff': args.auto_takeoff,
                          'auto_global_plan': args.auto_plan, 'auto_timeout': args.auto_timeout,
                          'low_vram': args.low_vram, 'macvo': not args.disable_macvo,
                          'random_walk': args.keep_random_walk, 'episode_seconds': args.episode_seconds,
                          'goal_radius_m': args.goal_radius, 'waypoint_radius_m': args.waypoint_radius,
                          'protocol': 'local_progress_validation',
                          'paper_exact_reproduction': False,
                          'goal_count': len(load_goals(args.env, args.task, args.pose)[0]) if args.episode_seconds else None},
                         ensure_ascii=False, indent=2))
        return 0
    # One launch owns the stack; release automatically on exit/crash.
    lock_dir = Path.home() / '.cache/raven'
    lock_dir.mkdir(parents=True, exist_ok=True)
    with (lock_dir / 'console.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有 RAVEN 镜像构建或统一启动器占用运行锁。\n'
                  '如果正在执行 build_raven_images.sh，请等它显示 [完成] 后再启动实验；'
                  '否则请回到原启动器终端。\n'
                  '--restart 只重启已有容器，不会中断正在运行的构建或启动器。',
                  file=sys.stderr)
            return 1
        console = Console(args)
        def interrupt(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupt)
        console.emit('SETUP', f'场景={args.env} 任务={args.task!r} 日志={console.run_dir}')
        console.emit('SETUP', 'Ctrl+C 会停止本次整套仿真；'
                     f'自动起飞={args.auto_takeoff}，自动跟随全局规划={args.auto_plan}。')
        code = 0
        try:
            console.setup()
            code = console.monitor() or 0
        except KeyboardInterrupt:
            code = 130
        except Exception as exc:
            console.emit('ALERT', str(exc))
            if console.episode:
                console.episode.finish('startup_or_supervisor_error', str(exc))
            code = 1
        finally:
            console.shutdown()
        return code


if __name__ == '__main__':
    sys.exit(main())
