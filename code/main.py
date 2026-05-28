import topography, fly_control, os, json, graphics, test, time, logging, queue
import numpy as np
import weather_client
import energy_model as em
from threads import HealthRegistry, PixhawkData, PixhawkThread

# ── Paramètres de planification ─────────────────────────────────────────────
N_PATHS        = 1     # nombre de graphes comparatifs (mode test)
PENALTY_MIN    = 0.5
PENALTY_MAX    = 0.5
EPSILON_ALT    = 0.5   # seuil de lissage (mètres)
# ────────────────────────────────────────────────────────────────────────────


# ════════════════════════════════════════════════════════════════════════════
#  Fonctions utilitaires de la boucle drone
# ════════════════════════════════════════════════════════════════════════════

def log_state(logger, lat, lon, altitude, battery):
    logger.info(
        f"STATE | lat={lat} lon={lon} alt={altitude}m battery={battery}%"
    )


def change_drone_state(logger, old_state, state):
    logger.info(f"CHANGING DRONE STATE | {old_state} -> {state}")
    return state


def get_data(capteurs=None):
    """
    Retourne les données capteurs.
    Si le Pixhawk est connecté (pixhawk_data fourni), le GPS et la batterie
    proviennent de la liaison MAVLink ; sinon on simule.
    """
    if capteurs is None:
        capteurs = ["camera_rgb", "camera_thermique", "gps", "sms"]
    data = {}
    if "camera_rgb"       in capteurs: data["camera_rgb"]       = np.random.rand(100, 100, 3)
    if "camera_thermique" in capteurs: data["camera_thermique"] = np.random.rand(100, 100)
    if "gps"              in capteurs: data["gps"]              = np.random.rand(2)
    if "sms"              in capteurs: data["sms"]              = np.random.rand(1)
    return data


def send_data(data):
    print(data)


# ════════════════════════════════════════════════════════════════════════════
#  Initialisation du drone (logging + config + threads)
# ════════════════════════════════════════════════════════════════════════════

def init_drone():
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)

    try:
        os.remove("../logs/drone.log")
    except Exception:
        pass

    try:
        logging.basicConfig(
            filename=r"../logs/drone.log",
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(message)s"
        )
    except Exception as e:
        logging.error(f"FATAL ERROR | \n{e}")
        exit()

    logger = logging.getLogger()
    logger.info("DEBUT DE MISSION")

    try:
        config_json_path = "config.json"
        config = json.load(open(config_json_path))
    except Exception as e:
        logger.error(f"FATAL ERROR | \n{e}")
        exit()

    current_map = topography.Map(config)

    # ── Démarrage du thread Pixhawk ──────────────────────────────────────
    health        = HealthRegistry()
    pixhawk_data  = PixhawkData()
    mission_queue = queue.Queue()

    pixhawk_thread = PixhawkThread(
        pixhawk_data  = pixhawk_data,
        mission_queue = mission_queue,
        health        = health,
    )
    pixhawk_thread.start()
    logger.info("[Main] Thread Pixhawk démarré")

    state      = change_drone_state(logger, None, 11)
    start_time = time.time()
    lat, lon   = 0, 0
    altitude   = 0   # hors test : l'altitude réelle viendra du Pixhawk

    main_loop(
        current_map, state, start_time, lat, lon, altitude, logger,
        pixhawk_data=pixhawk_data,
        mission_queue=mission_queue,
        pixhawk_thread=pixhawk_thread,
    )


# ════════════════════════════════════════════════════════════════════════════
#  Boucle principale du drone
# ════════════════════════════════════════════════════════════════════════════

def main_loop(current_map, state, start_time, lat, lon, altitude, logger,
              pixhawk_data: PixhawkData = None,
              mission_queue: queue.Queue = None,
              pixhawk_thread: PixhawkThread = None):
    """
    Boucle d'états du drone.

    États :
        10 : fin de mission
        11 : au sol
        12 : décollage
        13 : atterrissage
        14 : en vol
        15 : scan local
        40 : erreur indéterminée
        41 : retour d'urgence
        71 : retour d'urgence (variante)
        72 : atterrissage d'urgence
    """
    current_path       = []
    running_subprocess = []
    running            = True

    while running:
        try:
            # ── Lecture GPS + batterie depuis le Pixhawk ──────────────
            if pixhawk_data is not None:
                pix = pixhawk_data.snapshot()
                if pix["lat"] is not None:
                    lat, lon = pix["lat"], pix["lon"]
                if pix["relative_alt"] is not None:
                    altitude = pix["relative_alt"]
                battery = pix["battery_pct"]
            else:
                battery = None

            match state:
                case 11:
                    # Au sol — simulation de logs pour la démo
                    for i in range(10):
                        log_state(logger, i, i, i, 100 - i)
                    running = False

                case 12:
                    # Décollage
                    pass

                case 13:
                    # Atterrissage
                    pass

                case 14:
                    # En vol — log de l'état GPS/batterie réel
                    log_state(logger, lat, lon, altitude, battery)

                case 15:
                    # Scan local
                    pass

                case 40:
                    # Urgence indéterminée
                    pass

                case 41:
                    # Retour d'urgence
                    pass

                case _:
                    print("État inconnu")

        except Exception as e:
            logger.critical(f"loop crash: {e}", exc_info=True)
            state = change_drone_state(logger, state, 40)

    # ── Arrêt propre du thread Pixhawk ───────────────────────────────────
    if pixhawk_thread is not None:
        pixhawk_thread.stop()
        pixhawk_thread.join(timeout=3)
        logger.info("[Main] Thread Pixhawk arrêté")

    logger.info("FIN DE MISSION")


# ════════════════════════════════════════════════════════════════════════════
#  Point d'entrée principal — planification + affichage
# ════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)

    config_json_path = "config.json"
    config = json.load(open(config_json_path))

    current_map = topography.Map(config)

    # ── 1. Récupération des données météo ────────────────────────────────
    bounds     = current_map.dataset.bounds
    lat_center = (bounds.bottom + bounds.top)   / 2
    lon_center = (bounds.left   + bounds.right) / 2
    from pyproj import Transformer
    transformer_to_wgs84 = Transformer.from_crs(
        current_map.dataset.crs, "EPSG:4326", always_xy=True
    )
    lon_wgs84, lat_wgs84 = transformer_to_wgs84.transform(lon_center, lat_center)

    wind = weather_client.get_wind(lat_wgs84, lon_wgs84)
    print(f"\n[Météo] Vent : {wind['speed']:.1f} m/s, direction {wind['dir_deg']:.0f}°"
          f"  (u={wind['u']:+.2f} m/s Est, v={wind['v']:+.2f} m/s Nord)")

    # ── 2. Modèle énergétique ─────────────────────────────────────────────
    energy_mdl = em.DroneEnergyModel(config)
    print(f"[Énergie] Masse {energy_mdl.mass_kg} kg | "
          f"Batterie {energy_mdl.battery_wh} Wh | "
          f"Vitesse croisière {energy_mdl.cruise_speed} m/s | "
          f"Puissance survol {energy_mdl.hover_power_w} W")

    # ── 3. Génération des points de mission ───────────────────────────────
    start_point = current_map.get_random_valid_point()
    targets     = [current_map.get_random_valid_point() for _ in range(3)]
    print(f"\n[Mission] Départ : {start_point}")
    for i, t in enumerate(targets):
        print(f"          Cible {i+1} : {t}")

    # ── 4. Planification A* avec vent + énergie ───────────────────────────
    print("\n[Planification] Calcul des trajets en cours...")
    aller_path, retour_path = fly_control.boucle_principale(
        current_map,
        start_node=start_point,
        targets=targets,
        penalty=PENALTY_MIN,
        wind=wind,
        energy_mdl=energy_mdl
    )

    # ── 5. Bilan énergétique ──────────────────────────────────────────────
    if aller_path:
        bilan_aller = energy_mdl.total_path_energy(aller_path, wind)
        print(f"\n[Bilan aller]")
        print(f"  Waypoints   : {len(aller_path)}")
        print(f"  Énergie     : {bilan_aller['energy_j'] / 1000:.2f} kJ")
        print(f"  Batterie    : {bilan_aller['battery_%']:.1f} %")
        print(f"  Temps estimé: {bilan_aller['time_s'] / 60:.1f} min")

    if retour_path:
        bilan_retour = energy_mdl.total_path_energy(retour_path, wind)
        print(f"\n[Bilan retour]")
        print(f"  Waypoints   : {len(retour_path)}")
        print(f"  Énergie     : {bilan_retour['energy_j'] / 1000:.2f} kJ")
        print(f"  Batterie    : {bilan_retour['battery_%']:.1f} %")
        print(f"  Temps estimé: {bilan_retour['time_s'] / 60:.1f} min")

    if aller_path and retour_path:
        total_j   = bilan_aller['energy_j'] + bilan_retour['energy_j']
        total_pct = total_j / (energy_mdl.battery_wh * 3600) * 100
        total_min = (bilan_aller['time_s'] + bilan_retour['time_s']) / 60
        print(f"\n[Bilan total]  {total_j/1000:.2f} kJ — "
              f"{total_pct:.1f}% batterie — {total_min:.1f} min")
        if total_pct > 80:
            print("  ⚠ Attention : consommation > 80 % de la batterie !")

    # ── 6. Affichage graphique ────────────────────────────────────────────
    if aller_path:
        plot_paths  = [aller_path]
        plot_titles = [
            f"Aller | {len(aller_path)} wp | "
            f"{bilan_aller['energy_j']/1000:.1f} kJ | "
            f"{bilan_aller['time_s']/60:.1f} min"
        ]

        if retour_path:
            plot_paths.append(retour_path)
            plot_titles.append(
                f"Retour | {len(retour_path)} wp | "
                f"{bilan_retour['energy_j']/1000:.1f} kJ | "
                f"{bilan_retour['time_s']/60:.1f} min"
            )

        graphics.mutliplot_path(
            current_map,
            plot_paths,
            plot_titles,
            targets_list=[targets, []],
            wind=wind
        )

    # ── 7. Démarrage de la boucle drone (avec thread Pixhawk) ────────────
    init_drone()
