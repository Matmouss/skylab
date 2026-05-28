import threading
import queue
import time
import logging

logger = logging.getLogger(__name__)

# ── Port série UART (TX du Pi → RX du Pixhawk, RX du Pi → TX du Pixhawk) ──
PIXHAWK_SERIAL_PORT = "/dev/serial0"   # ou /dev/ttyAMA0 selon le Pi
PIXHAWK_BAUD_RATE   = 57600            # baud par défaut MAVLink sur Pixhawk


# ════════════════════════════════════════════════════════════════════════════
#  HealthRegistry  (inchangé — utilisé par tous les threads)
# ════════════════════════════════════════════════════════════════════════════

class HealthRegistry:
    def __init__(self):
        self._lock = threading.Lock()
        self._heartbeat = {}
        self._progress  = {}

    def register(self, name):
        now = time.monotonic()
        with self._lock:
            self._heartbeat[name] = now
            self._progress[name]  = 0

    def progress(self, name):
        with self._lock:
            self._heartbeat[name] = time.monotonic()
            self._progress[name] += 1

    def snapshot(self):
        with self._lock:
            return dict(self._heartbeat), dict(self._progress)


# ════════════════════════════════════════════════════════════════════════════
#  PixhawkData  — données partagées lues depuis le Pixhawk
# ════════════════════════════════════════════════════════════════════════════

class PixhawkData:
    """
    Conteneur thread-safe pour les données reçues du Pixhawk via MAVLink.

    Accès depuis le thread principal :
        data = pixhawk_data.snapshot()
        lat, lon = data["lat"], data["lon"]
        battery  = data["battery_pct"]
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            "lat":         None,   # degrés décimaux
            "lon":         None,
            "alt":         None,   # mètres (altitude barométrique)
            "relative_alt": None,  # mètres au-dessus du home
            "heading":     None,   # degrés (0-360)
            "battery_pct": None,   # pourcentage 0-100
            "battery_v":   None,   # tension en volts
            "armed":       None,   # bool
            "flight_mode": None,   # str (ex. "GUIDED", "STABILIZE"…)
            "last_update": None,   # time.monotonic()
        }

    def update(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
            self._state["last_update"] = time.monotonic()

    def snapshot(self):
        with self._lock:
            return dict(self._state)


# ════════════════════════════════════════════════════════════════════════════
#  Thread Pixhawk
# ════════════════════════════════════════════════════════════════════════════

class PixhawkThread(threading.Thread):
    """
    Thread dédié à la communication MAVLink avec le Pixhawk via UART.

    Fonctions :
        • Réception continue des messages MAVLink (GPS, batterie, état…)
        • Mise à jour de PixhawkData (partagé avec le thread principal)
        • Envoi de nouvelles missions (waypoints) depuis une queue

    Paramètres
    ----------
    pixhawk_data  : PixhawkData   — objet partagé mis à jour en continu
    mission_queue : queue.Queue   — file de missions à envoyer au Pixhawk
                                    Chaque item = liste de (lat, lon, alt)
    health        : HealthRegistry | None
    port          : str           — port série (défaut : PIXHAWK_SERIAL_PORT)
    baud          : int           — baud rate  (défaut : PIXHAWK_BAUD_RATE)
    """

    THREAD_NAME = "pixhawk"

    def __init__(
        self,
        pixhawk_data:  PixhawkData,
        mission_queue: queue.Queue,
        health:        HealthRegistry = None,
        port:          str = PIXHAWK_SERIAL_PORT,
        baud:          int = PIXHAWK_BAUD_RATE,
    ):
        super().__init__(name=self.THREAD_NAME, daemon=True)
        self.pixhawk_data  = pixhawk_data
        self.mission_queue = mission_queue
        self.health        = health
        self.port          = port
        self.baud          = baud
        self._stop_event   = threading.Event()

    def stop(self):
        self._stop_event.set()

    # ── Boucle principale ────────────────────────────────────────────────

    def run(self):
        if self.health:
            self.health.register(self.THREAD_NAME)

        try:
            from pymavlink import mavutil
        except ImportError:
            logger.error(
                "[Pixhawk] pymavlink non installé — "
                "lancez : pip install pymavlink"
            )
            return

        logger.info(
            f"[Pixhawk] Connexion sur {self.port} @ {self.baud} baud…"
        )

        try:
            mav = mavutil.mavlink_connection(
                self.port,
                baud=self.baud,
                source_system=255,  # GCS
            )
            mav.wait_heartbeat(timeout=10)
            logger.info(
                f"[Pixhawk] Heartbeat reçu — "
                f"système {mav.target_system} / composant {mav.target_component}"
            )
        except Exception as e:
            logger.error(f"[Pixhawk] Échec de connexion : {e}")
            return

        # Demande les flux de données principaux
        self._request_data_streams(mav)

        while not self._stop_event.is_set():
            try:
                # ── Réception MAVLink (non-bloquant) ───────────────────
                msg = mav.recv_match(blocking=True, timeout=0.5)
                if msg:
                    self._handle_message(msg)
                    if self.health:
                        self.health.progress(self.THREAD_NAME)

                # ── Envoi de mission si disponible ─────────────────────
                if not self.mission_queue.empty():
                    try:
                        waypoints = self.mission_queue.get_nowait()
                        self._send_mission(mav, waypoints)
                    except queue.Empty:
                        pass

            except Exception as e:
                logger.warning(f"[Pixhawk] Erreur dans la boucle : {e}")
                time.sleep(0.1)

        logger.info("[Pixhawk] Thread arrêté.")

    # ── Traitement des messages MAVLink ──────────────────────────────────

    def _handle_message(self, msg):
        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            self.pixhawk_data.update(
                lat         = msg.lat  / 1e7,   # degrés décimaux
                lon         = msg.lon  / 1e7,
                alt         = msg.alt  / 1e3,   # mm → m
                relative_alt= msg.relative_alt / 1e3,
                heading     = msg.hdg / 100.0,  # cdeg → deg
            )

        elif msg_type == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0  # mV → V
            remaining = msg.battery_remaining        # -1 si inconnu
            self.pixhawk_data.update(
                battery_v   = voltage,
                battery_pct = remaining if remaining >= 0 else None,
            )

        elif msg_type == "HEARTBEAT":
            from pymavlink import mavutil
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode_map = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO",
                        4: "GUIDED",    5: "LOITER",   6: "RTL",
                        9: "LAND",     16: "POSHOLD"}
            flight_mode = mode_map.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
            self.pixhawk_data.update(armed=armed, flight_mode=flight_mode)

    # ── Flux de données ─────────────────────────────────────────────────

    def _request_data_streams(self, mav):
        """Demande les flux MAVLink essentiels au Pixhawk."""
        streams = [
            (mav.mavlink.MAV_DATA_STREAM_POSITION,   2),   # GPS @ 2 Hz
            (mav.mavlink.MAV_DATA_STREAM_EXTRA1,     5),   # Attitude @ 5 Hz
            (mav.mavlink.MAV_DATA_STREAM_EXTRA2,     2),   # VFR_HUD @ 2 Hz
        ]
        for stream_id, rate in streams:
            mav.mav.request_data_stream_send(
                mav.target_system,
                mav.target_component,
                stream_id,
                rate,
                1,   # 1 = démarrer
            )

    # ── Envoi de mission ─────────────────────────────────────────────────

    def _send_mission(self, mav, waypoints):
        """
        Envoie une liste de waypoints au Pixhawk via le protocole MAVLink
        mission upload.

        waypoints : liste de (lat, lon, alt)  — lat/lon en degrés décimaux,
                    alt en mètres (relatif au home)
        """
        from pymavlink import mavutil
        n = len(waypoints)
        logger.info(f"[Pixhawk] Envoi mission : {n} waypoints")

        try:
            # ── 1. Annonce le nombre de waypoints ─────────────────────
            mav.mav.mission_count_send(
                mav.target_system,
                mav.target_component,
                n,
            )

            seq = 0
            while seq < n:
                # ── 2. Attente MISSION_REQUEST ──────────────────────
                req = mav.recv_match(
                    type="MISSION_REQUEST", blocking=True, timeout=5
                )
                if req is None:
                    logger.error("[Pixhawk] Timeout MISSION_REQUEST")
                    return
                seq = req.seq
                lat, lon, alt = waypoints[seq]

                mav.mav.mission_item_send(
                    mav.target_system,
                    mav.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,    # current = 0
                    1,    # autocontinue
                    0, 0, 0, 0,          # param 1-4 (hold, accept radius…)
                    lat, lon, alt,
                )
                seq += 1

            # ── 3. Confirmation ────────────────────────────────────
            ack = mav.recv_match(
                type="MISSION_ACK", blocking=True, timeout=5
            )
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.info("[Pixhawk] Mission acceptée ✓")
            else:
                logger.warning(f"[Pixhawk] Mission refusée : {ack}")

        except Exception as e:
            logger.error(f"[Pixhawk] Erreur lors de l'envoi de la mission : {e}")
