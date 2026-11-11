#!/usr/bin/env python3
"""Integration smoke tests for velocity_manager."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


ROOT = Path(__file__).resolve().parents[3]  # test_ws


class Tester(Node):
    def __init__(self) -> None:
        super().__init__('velocity_manager_smoke_tester')
        self.msgs: list[Twist] = []
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.switch_cli = self.create_client(Trigger, '/vel_mng/switch')
        self.enable_cli = self.create_client(SetBool, '/vel_mng/enable')

    def _on_cmd(self, msg: Twist) -> None:
        self.msgs.append(msg)

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_services(self, timeout: float = 10.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if self.switch_cli.service_is_ready() and self.enable_cli.service_is_ready():
                return
            self.spin_for(0.1)
        raise RuntimeError('services not ready')

    def wait_publisher(self, timeout: float = 10.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if self.count_publishers('cmd_vel') >= 1:
                return
            self.spin_for(0.1)
        raise RuntimeError('no publisher on cmd_vel')

    def call_enable(self, data: bool):
        req = SetBool.Request()
        req.data = data
        fut = self.enable_cli.call_async(req)
        end = time.time() + 5.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError('enable service timeout')
        return fut.result()

    def call_switch(self):
        req = Trigger.Request()
        fut = self.switch_cli.call_async(req)
        end = time.time() + 5.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError('switch service timeout')
        return fut.result()

    def publish_input(self, topic: str, x: float) -> None:
        pub = self.create_publisher(Twist, topic, 10)
        # wait for match
        end = time.time() + 3.0
        while time.time() < end and self.count_subscribers(topic) < 1:
            self.spin_for(0.05)
        msg = Twist()
        msg.linear.x = x
        before = len(self.msgs)
        for _ in range(5):
            pub.publish(msg)
            self.spin_for(0.1)
        self.destroy_publisher(pub)
        return before


def pass_(msg: str) -> None:
    print(f'[PASS] {msg}')


def fail(msg: str) -> None:
    raise AssertionError(msg)


def start_node(env: dict) -> subprocess.Popen:
    return subprocess.Popen(
        ['ros2', 'launch', 'velocity_manager', 'velocity_manager.launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        preexec_fn=os.setsid,
    )


def stop_node(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=2)


def main() -> int:
    env = os.environ.copy()
    # Ensure overlay is active when launched from scripts
    setup = ROOT / 'install' / 'setup.bash'
    if not setup.exists():
        print(f'missing {setup}; build the package first', file=sys.stderr)
        return 1

    proc = None
    rclpy.init()
    tester = Tester()
    try:
        print('=== Starting velocity_manager ===')
        proc = start_node(env)
        tester.wait_services()
        tester.wait_publisher()
        tester.spin_for(0.5)

        print('=== Case 1: disabled blocks output ===')
        tester.msgs.clear()
        tester.publish_input('/nav/cmd_vel', 1.0)
        tester.spin_for(0.5)
        if any(abs(m.linear.x - 1.0) < 1e-6 for m in tester.msgs):
            fail('output should be blocked while disabled')
        pass_('disabled blocks passthrough')

        print('=== Case 2: enable false publishes zero once ===')
        tester.msgs.clear()
        tester.spin_for(0.2)
        res = tester.call_enable(False)
        tester.spin_for(0.5)
        if not res.success:
            fail(f'enable false failed: {res}')
        if not any(abs(m.linear.x) < 1e-9 and abs(m.angular.z) < 1e-9 for m in tester.msgs):
            fail(f'expected zero Twist on disable, got {len(tester.msgs)} msgs')
        pass_('disable publishes zero Twist')

        print('=== Case 3: enable true + selected source passthrough ===')
        res = tester.call_enable(True)
        if not res.success:
            fail(f'enable true failed: {res}')
        tester.msgs.clear()
        tester.publish_input('/nav/cmd_vel', 1.5)
        tester.spin_for(0.3)
        if not any(abs(m.linear.x - 1.5) < 1e-6 for m in tester.msgs):
            fail('expected passthrough from navigation')
        pass_('enabled passthrough from default source')

        print('=== Case 4: non-selected source is ignored ===')
        tester.msgs.clear()
        tester.publish_input('/joy/cmd_vel', 9.9)
        tester.spin_for(0.3)
        if any(abs(m.linear.x - 9.9) < 1e-6 for m in tester.msgs):
            fail('non-selected joystick should not passthrough')
        pass_('non-selected source ignored')

        print('=== Case 5: switch rotates sources ===')
        res = tester.call_switch()
        if 'joystick' not in res.message:
            fail(f'expected joystick after first switch: {res.message}')
        tester.msgs.clear()
        tester.publish_input('/joy/cmd_vel', 2.2)
        tester.spin_for(0.3)
        if not any(abs(m.linear.x - 2.2) < 1e-6 for m in tester.msgs):
            fail('expected passthrough from joystick after switch')
        pass_('switch to joystick works')

        res = tester.call_switch()
        if 'keyboard' not in res.message:
            fail(f'expected keyboard: {res.message}')
        res = tester.call_switch()
        if 'navigation' not in res.message:
            fail(f'expected wrap to navigation: {res.message}')
        pass_('switch wraps around')

        print('=== Case 6: switch while disabled does not resume output ===')
        tester.call_enable(False)
        tester.spin_for(0.2)
        tester.msgs.clear()
        res = tester.call_switch()
        if 'joystick' not in res.message:
            fail(f'switch while disabled should still rotate: {res.message}')
        tester.publish_input('/joy/cmd_vel', 3.3)
        tester.spin_for(0.3)
        if any(abs(m.linear.x - 3.3) < 1e-6 for m in tester.msgs):
            fail('switch while disabled must not resume output')
        pass_('switch while disabled keeps block')

        print('=== Case 7: re-enable restores previous selected source ===')
        tester.call_enable(True)
        tester.msgs.clear()
        tester.publish_input('/joy/cmd_vel', 4.4)
        tester.spin_for(0.3)
        if not any(abs(m.linear.x - 4.4) < 1e-6 for m in tester.msgs):
            fail('expected resume with joystick')
        pass_('re-enable keeps selected source')

        print('=== Case 8: invalid default_input fails startup ===')
        stop_node(proc)
        proc = None
        with tempfile.TemporaryDirectory() as tmp:
            invalid = Path(tmp) / 'invalid.yaml'
            invalid.write_text(
                """
velocity_manager:
  ros__parameters:
    output_topic: "cmd_vel"
    default_input: "does_not_exist"
    input_names: ["navigation"]
    input_topics: ["/nav/cmd_vel"]
"""
            )
            bad = subprocess.run(
                [
                    'ros2', 'run', 'velocity_manager', 'velocity_manager_node',
                    '--ros-args', '--params-file', str(invalid),
                ],
                capture_output=True,
                text=True,
                env=env,
                timeout=8,
            )
            if bad.returncode == 0:
                fail('invalid default_input should fail startup')
            combined = (bad.stdout or '') + (bad.stderr or '')
            if 'default_input' not in combined and 'Failed to start' not in combined:
                fail(f'expected failure message, got: {combined}')
        pass_('invalid default_input fails startup')

        print()
        print('All smoke tests passed.')
        return 0
    finally:
        stop_node(proc)
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f'[FAIL] {exc}', file=sys.stderr)
        sys.exit(1)
