import threading
import queue
import time
import logging
import subprocess
import os
import signal
import json
import urllib.request

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
        
    def unregister(self, name):
        with self._lock:
            self._heartbeat.pop(name, None)
            self._progress.pop(name, None)
        
def fourg_thread(stop_event, health, rx_queue, send_queue, logger=None, config=None):
    name = "fourg"
    health.register(name)

    logger = logger or logging.getLogger()
    config = config or {}

    cfg = config.get("4g", {})

    command = cfg.get("command", ["sudo", "pppd", "call", "gprs"])
    interface = cfg.get("interface", "ppp0")
    ping_host = cfg.get("ping_host", "8.8.8.8")
    startup_delay = cfg.get("startup_delay", 30)
    check_period = cfg.get("check_period", 10)
    fail_limit = cfg.get("fail_limit", 3)
    restart_delay = cfg.get("restart_delay", 5)
    send_timeout = cfg.get("send_timeout", 5)
    max_send_retries = cfg.get("max_send_retries", 3)

    webhook = config.get("webhook")

    proc = None
    fail_count = 0
    last_check = 0
    started_at = 0

    logger.info("thread started")

    def start_4g():
        nonlocal proc, fail_count, last_check, started_at

        logger.info(f"4G START | {' '.join(command)}")

        proc = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        fail_count = 0
        started_at = time.monotonic()
        last_check = started_at

        rx_queue.put({
            "type": "4g",
            "status": "starting"
        })

    def stop_4g():
        nonlocal proc

        if proc and proc.poll() is None:
            logger.warning("4G STOP | kill pppd")

            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception as e:
                    logger.error(f"4G STOP ERROR | {e}")

        proc = None

    def interface_ok():
        r = subprocess.run(
            ["ip", "-4", "addr", "show", interface],
            capture_output=True,
            text=True
        )
        return r.returncode == 0 and "inet " in r.stdout

    def ping_ok():
        r = subprocess.run(
            ["ping", "-I", interface, "-c", "1", "-W", "3", ping_host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return r.returncode == 0

    def connection_ok():
        if time.monotonic() - started_at < startup_delay:
            return True

        return interface_ok() and ping_ok()

    def send_webhook(msg):
        if not webhook:
            logger.warning(f"4G SEND SKIPPED | no webhook | {msg}")
            return False

        data = json.dumps(msg).encode("utf-8")

        req = urllib.request.Request(
            webhook,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST"
        )

        with urllib.request.urlopen(req, timeout=send_timeout) as r:
            return 200 <= r.status < 300

    def flush_send_queue():
        while not send_queue.empty():
            msg = send_queue.get()

            attempts = msg.get("_send_attempts", 0)

            try:
                ok = send_webhook(msg)

                if ok:
                    logger.info(f"4G SEND OK | {msg}")

                    rx_queue.put({
                        "type": "4g_send",
                        "status": "sent",
                        "message": msg
                    })

                else:
                    raise RuntimeError("webhook returned bad status")

            except Exception as e:
                attempts += 1
                msg["_send_attempts"] = attempts

                logger.warning(
                    f"4G SEND FAIL | attempt={attempts}/{max_send_retries} | {e}"
                )

                if attempts < max_send_retries:
                    send_queue.put(msg)
                else:
                    rx_queue.put({
                        "type": "4g_send",
                        "status": "failed",
                        "message": msg,
                        "error": str(e)
                    })

                break

    def restart_4g():
        nonlocal fail_count

        rx_queue.put({
            "type": "4g",
            "status": "restarting"
        })

        stop_4g()
        time.sleep(restart_delay)
        start_4g()
        fail_count = 0

    try:
        start_4g()

        while not stop_event.is_set():
            now = time.monotonic()

            if proc and proc.poll() is not None:
                logger.error(f"4G DEAD | pppd exited rc={proc.returncode}")
                restart_4g()

            flush_send_queue()

            if now - last_check >= check_period:
                last_check = now

                if connection_ok():
                    fail_count = 0

                    rx_queue.put({
                        "type": "4g",
                        "status": "connected"
                    })

                    logger.info("4G OK")

                else:
                    fail_count += 1
                    logger.warning(f"4G FAIL | {fail_count}/{fail_limit}")

                    if fail_count >= fail_limit:
                        restart_4g()

            health.progress(name)
            time.sleep(1)

    except Exception as e:
        logger.exception(f"{name} crash: {e}")

    finally:
        stop_4g()
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

def watchdog_thread(global_stop_event, health, threads, worker_stop_events, worker_factories, alert_queue, logger=None):
    timeouts = {
        "main": 1.0,
        "fourg": 20.0,
        "gps": 2.0,
    }

    restartable = {"fourg", "gps"}

    last_progress = {}
    restart_count = {}

    startup_grace = 3.0
    restart_join_timeout = 3.0
    max_restart = 10

    logger = logger or logging.getLogger()
    logger.info("thread started")

    start_time = time.monotonic()

    def stop_main(reason):
        msg = f"MAIN FAILURE | {reason} | full program restart required"
        logger.critical(msg)

        alert_queue.put({
            "type": "watchdog",
            "action": "stop_main",
            "reason": reason
        })

        global_stop_event.set()

    def restart_worker(name, reason):
        if name not in restartable:
            stop_main(f"{name} failed and is not restartable: {reason}")
            return

        restart_count[name] = restart_count.get(name, 0) + 1

        if restart_count[name] > max_restart:
            stop_main(f"{name} too many restarts: {restart_count[name]}")
            return

        logger.error(f"RESTART THREAD | {name} | reason={reason}")

        try:
            worker_stop_events[name].set()
        except Exception:
            pass

        old_thread = threads.get(name)

        if old_thread and old_thread.is_alive():
            old_thread.join(timeout=restart_join_timeout)

        if old_thread and old_thread.is_alive():
            logger.error(f"{name} did not stop cleanly, abandoned as daemon")

        try:
            health.unregister(name)
        except Exception:
            pass

        worker_stop_events[name] = threading.Event()

        new_thread = worker_factories[name](worker_stop_events[name])
        threads[name] = new_thread
        new_thread.start()

        last_progress.pop(name, None)

        alert_queue.put({
            "type": "watchdog",
            "action": "restart",
            "thread": name,
            "reason": reason,
            "count": restart_count[name],
        })

    while not global_stop_event.is_set():
        try:
            hb, prog = health.snapshot()
            now = time.monotonic()

            for name, timeout in timeouts.items():
                if name not in hb:
                    if now - start_time < startup_grace:
                        continue

                    if name == "main":
                        stop_main("main not registered")
                        return

                    restart_worker(name, "not registered")
                    continue

                if name in threads and not threads[name].is_alive():
                    restart_worker(name, "thread dead")
                    continue

                if now - hb[name] > timeout:
                    if name == "main":
                        stop_main("main heartbeat timeout")
                        return

                    restart_worker(name, "heartbeat timeout")
                    continue

                if name in last_progress and prog[name] == last_progress[name]:
                    if now - hb[name] > timeout / 2:
                        if name == "main":
                            stop_main("main no progress")
                            return

                        restart_worker(name, "no progress")
                        continue

                last_progress[name] = prog[name]

            time.sleep(0.2)

        except Exception as e:
            logger.exception(f"watchdog crash: {e}")
            alert_queue.put(f"watchdog crash: {e}")
            global_stop_event.set()
            return

    logger.info("thread stopped")

def start_background_threads(logger=None, config=None):
    global_stop_event = threading.Event()

    health = HealthRegistry()

    rx_queue = queue.Queue()
    send_queue = queue.Queue()
    gps_queue = queue.Queue()
    alert_queue = queue.Queue()

    logger = logger or logging.getLogger()

    worker_stop_events = {
        "fourg": threading.Event(),
        "gps": threading.Event(),
    }

    def make_fourg_thread(stop_event):
        return threading.Thread(
            target=fourg_thread,
            args=(stop_event, health, rx_queue, send_queue, logger, config),
            daemon=True,
            name="fourg",
        )

    def make_gps_thread(stop_event):
        return threading.Thread(
            target=gps_thread,
            args=(stop_event, health, gps_queue, logger),
            daemon=True,
            name="gps",
        )

    worker_factories = {
        "fourg": make_fourg_thread,
        "gps": make_gps_thread,
    }

    workers = {
        "fourg": make_fourg_thread(worker_stop_events["fourg"]),
        "gps": make_gps_thread(worker_stop_events["gps"]),
    }

    for t in workers.values():
        t.start()

    watchdog = threading.Thread(
        target=watchdog_thread,
        args=(
            global_stop_event,
            health,
            workers,
            worker_stop_events,
            worker_factories,
            alert_queue,
            logger,
        ),
        daemon=True,
        name="watchdog",
    )
    watchdog.start()

    logger.info("all threads started")

    return {
        "stop_event": global_stop_event,
        "health": health,
        "rx_queue": rx_queue,
        "send_queue": send_queue,
        "gps_queue": gps_queue,
        "alert_queue": alert_queue,
        "workers": workers,
        "worker_stop_events": worker_stop_events,
        "worker_factories": worker_factories,
        "watchdog": watchdog,
    }

def stop_background_threads(thread_ctx):
    logger = logging.getLogger()
    logger.info("stopping threads")

    thread_ctx["stop_event"].set()

    for ev in thread_ctx["worker_stop_events"].values():
        ev.set()

    for t in thread_ctx["workers"].values():
        t.join(timeout=2)

    thread_ctx["watchdog"].join(timeout=2)

    logger.info("all threads stopped")
    