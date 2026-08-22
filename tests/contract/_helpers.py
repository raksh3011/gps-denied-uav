"""Shared subprocess-node test infrastructure for tests/contract/*.py.

Not a test module itself (leading underscore keeps pytest from collecting
it). Every test_*_contracts.py file in this directory used to carry its
own byte-for-byte copy of RunningNode/settle_and_clear/the QoS profiles
below — this is that logic in one place instead of five.
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
MISSION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

SETTLE_SECONDS = 1.5


class RunningNode:
    """Launches a compiled node as a subprocess and tears it down.

    Tests share fixed topic names, so a message published by one test's
    node right before it's torn down can still be delivered after the
    next test's subscriber comes up — see settle_and_clear() below.
    """

    def __init__(self, package: str, executable: str, *args: str):
        # start_new_session puts the ros2-run wrapper AND the node binary
        # in one process group we can kill together — terminating only
        # the wrapper orphans the node, which then keeps publishing into
        # every later test in the file.
        self.proc = subprocess.Popen(
            ['ros2', 'run', package, executable, *args],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)

    def stop(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait()


def settle_and_clear(nodes, *collectors, seconds=SETTLE_SECONDS):
    """Spin for `seconds` to let discovery/queued messages flush, then
    discard anything collected so far — the real test window starts clean.
    """
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)
    for c in collectors:
        c.clear()
