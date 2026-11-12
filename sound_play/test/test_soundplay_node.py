"""Locking in soundplay_node: the idle timer must not hold the node's mutex.

The timer callback (spin_once -> idle_loop -> cleanup) used to run under
self.mutex while cleanup() took the same non-reentrant lock again, so the node
deadlocked as soon as a sound was left in the cache when the timer fired -- in
practice right after the first sound_play action goal. After that no goal and
no robotsound message was processed.

These tests call the node's own methods with stand-in sounds, so they need no
audio device.
"""

import importlib.util
import os
import threading
import time

import pytest
import rclpy

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'soundplay_node.py')


@pytest.fixture(scope='module')
def soundplay():
    spec = importlib.util.spec_from_file_location('soundplay_node', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def shared_node(soundplay):
    rclpy.init()
    n = soundplay.SoundPlayNode()
    n.destroy_timer(n.timer)        # the tests call spin_once themselves
    n.init_vars()                   # create the sound caches once
    n.init_vars = lambda: None      # then keep what a test puts in them
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def node(shared_node):
    shared_node.builtinsounds = {}
    shared_node.filesounds = {}
    shared_node.voicesounds = {}
    return shared_node


class FakeSound:
    """A cached sound; stale (purgeable) once ``stale_at`` has passed."""

    def __init__(self, stale_at=0.0):
        self.stale_at = stale_at

    def get_staleness(self):
        return 0 if time.monotonic() < self.stale_at else 100

    def dispose(self):
        pass

    def stop(self):
        pass


def run_in_thread(fn):
    t = threading.Thread(target=fn, daemon=True)
    t.start()
    return t


def test_idle_timer_with_a_cached_sound_returns(node):
    node.voicesounds = {'hello': FakeSound()}
    t = run_in_thread(node.spin_once)
    t.join(timeout=5.0)
    assert not t.is_alive(), 'spin_once deadlocked in cleanup()'
    assert node.voicesounds == {}


def test_idle_timer_does_not_hold_the_mutex_while_it_waits(node):
    # A sound still playing for 3 s: idle_loop sleeps and polls meanwhile.
    node.voicesounds = {'hello': FakeSound(stale_at=time.monotonic() + 3.0)}
    t = run_in_thread(node.spin_once)
    time.sleep(0.3)
    got = node.mutex.acquire(timeout=0.5)
    if got:
        node.mutex.release()
    t.join(timeout=10.0)
    assert got, 'execute_cb / robotsound would wait for the whole idle loop'
    assert not t.is_alive()
