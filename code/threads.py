import threading
import queue
import time


class HealthRegistry:
    def __init__(self):
        self._lock = threading.Lock()
        self._heartbeat = {}
        self._progress = {}

    def register(self, name):
        now = time.monotonic()
        with self._lock:
            self._heartbeat[name] = now
            self._progress[name] = 0

    def progress(self, name):
        with self._lock:
            self._heartbeat[name] = time.monotonic()
            self._progress[name] += 1

    def snapshot(self):
        with self._lock:
            return dict(self._heartbeat), dict(self._progress)