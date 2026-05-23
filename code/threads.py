import threading
import queue
import time
import logging
import random

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


def modem_thread(stop_event, health, rx_queue, send_queue, logger=None):
    name = "modem"
    health.register(name)

    logger = logger or logging.getLogger()

    logger.info("thread started")

    while not stop_event.is_set():
        try:
            while not send_queue.empty():
                msg = send_queue.get()
                logger.info(f"SMS ENVOYE | {msg}")

            if random.random() < 0.1:
                msg = {"type": "sms", "data": "test"}
                rx_queue.put(msg)

            while not rx_queue.empty():
                msg = rx_queue.get()
                logger.info(f"SMS RECU | {msg}")

            health.progress(name)

            time.sleep(0.5)

        except Exception as e:
            logger.exception(f"{name} crash: {e}")
            time.sleep(0.1)

    logger.info("thread stopped")


def gps_thread(stop_event, health, gps_queue, logger=None):
    name = "gps"
    health.register(name)

    logger = logger or logging.getLogger()

    logger.info("thread started")

    while not stop_event.is_set():
        try:
            # ici remplacer par la vraie lecture GPS
            time.sleep(0.5)

            gps_data = {"lat": 0.0, "lon": 0.0}
            gps_queue.put(gps_data)

            logger.info(f"gps update: {gps_data}")

            health.progress(name)

        except Exception as e:
            logger.exception(f"{name} crash: {e}")
            time.sleep(0.1)

    logger.info("thread stopped")

def watchdog_thread(stop_event, health, threads, alert_queue, logger=None):
    timeouts = {
        "main": 1.0,
        "modem": 2.0,
        "gps": 2.0,
    }

    last_progress = {}
    startup_grace = 2.0

    logger = logger or logging.getLogger()
    logger.info("thread started")

    start_time = time.monotonic()

    while not stop_event.is_set():
        try:
            hb, prog = health.snapshot()
            now = time.monotonic()

            for name, timeout in timeouts.items():
                if name not in hb:
                    if now - start_time < startup_grace:
                        continue
                    msg = f"{name} not registered"
                    logger.error(msg)
                    alert_queue.put(msg)
                    stop_event.set()
                    return

                if name in threads and not threads[name].is_alive():
                    msg = f"{name} thread dead"
                    logger.error(msg)
                    alert_queue.put(msg)
                    stop_event.set()
                    return

                if now - hb[name] > timeout:
                    msg = f"{name} heartbeat timeout"
                    logger.error(msg)
                    alert_queue.put(msg)
                    stop_event.set()
                    return

                if name in last_progress and prog[name] == last_progress[name]:
                    if now - hb[name] > timeout / 2:
                        msg = f"{name} no progress"
                        logger.error(msg)
                        alert_queue.put(msg)
                        stop_event.set()
                        return

                last_progress[name] = prog[name]

            time.sleep(0.2)

        except Exception as e:
            logger.exception(f"watchdog crash: {e}")
            alert_queue.put(f"watchdog crash: {e}")
            stop_event.set()
            return

    logger.info("thread stopped")


def start_background_threads(logger=None):
    stop_event = threading.Event()

    health = HealthRegistry()

    rx_queue = queue.Queue()
    send_queue = queue.Queue()
    gps_queue = queue.Queue()
    alert_queue = queue.Queue()

    logger = logger or logging.getLogger()

    workers = {
        "modem": threading.Thread(
            target=modem_thread,
            args=(stop_event, health, rx_queue, send_queue,logger),
            daemon=True,
            name="modem",
        ),
        "gps": threading.Thread(
            target=gps_thread,
            args=(stop_event, health, gps_queue, logger),
            daemon=True,
            name="gps",
        ),
    }

    for t in workers.values():
        t.start()

    watchdog = threading.Thread(
        target=watchdog_thread,
        args=(stop_event, health, workers, alert_queue, logger),
        daemon=True,
        name="watchdog",
    )
    watchdog.start()

    logger.info("all threads started")

    return {
        "stop_event": stop_event,
        "health": health,
        "rx_queue": rx_queue,
        "send_queue": send_queue,
        "gps_queue": gps_queue,
        "alert_queue": alert_queue,
        "workers": workers,
        "watchdog": watchdog,
    }


def stop_background_threads(thread_ctx):
    logger = logging.getLogger()
    logger.info("stopping threads")

    thread_ctx["stop_event"].set()

    for t in thread_ctx["workers"].values():
        t.join(timeout=1)

    thread_ctx["watchdog"].join(timeout=1)

    logger.info("all threads stopped")