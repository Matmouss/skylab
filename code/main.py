import topography, fly_control, os, json, graphics, test, time, logging, threads, queue
import numpy as np
import weather_client
import energy_model as em

N_PATHS        = 1     # nombre de graphes comparatifs (mode test)
PENALTY_MIN    = 0.5
PENALTY_MAX    = 0.5
EPSILON_ALT    = 0.5   # seuil de lissage (mètres)

def log_state(logger, lat, lon, altitude, battery):
    logger.info(
        f"STATE | lat={lat} lon={lon} alt={altitude}m battery={battery}%"
    )

def change_drone_state(logger, old_state,state):
    logger.info(f"CHANGING DRONE STATE | {old_state} -> {state}")
    return state

def build_state_ping(config, state, start_time, lat, lon, altitude, battery, current_path, latest_gps):
    return {
        "type": "ping",
        "drone_id": config.get("drone_id", "unknown"),
        "timestamp": time.time(),
        "uptime_s": round(time.monotonic() - start_time, 2),
        "state": state,
        "position": {
            "lat": lat,
            "lon": lon,
            "altitude": altitude
        },
        "battery": battery,
        "mission": {
            "path_len": len(current_path),
            "has_path": len(current_path) > 0
        },
        "gps": latest_gps
    }


# ════════════════════════════════════════════════════════════════════════════
#  Planification de mission (météo + A* + énergie + affichage)
# ════════════════════════════════════════════════════════════════════════════

def plan_mission(current_map, config):
    """
    Calcule le trajet aller/retour avec vent et modèle énergétique,
    affiche le bilan et les graphiques, et retourne les chemins planifiés.

    Retourne
    --------
    aller_path  : list de (x_proj, y_proj)
    retour_path : list de (x_proj, y_proj)
    wind        : dict vent utilisé pour la planification
    energy_mdl  : DroneEnergyModel instancié
    """
    # ── 1. Récupération des données météo ────────────────────────────────
    # Centre approximatif de la zone de vol
    bounds     = current_map.dataset.bounds
    lat_center = (bounds.bottom + bounds.top)   / 2
    lon_center = (bounds.left   + bounds.right) / 2
    # Conversion vers WGS84 pour l'API météo
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
    bilan_aller  = None
    bilan_retour = None

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

    return aller_path, retour_path, wind, energy_mdl


# ════════════════════════════════════════════════════════════════════════════
#  Initialisation du drone (logging + config)
# ════════════════════════════════════════════════════════════════════════════

def init_drone():
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)

    try:
        os.remove("../logs/drone.log")
    except:
        pass

    try:
        logging.basicConfig(
            filename=r"../logs/drone.log",
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(threadName)s | %(message)s"
        )
    except Exception as e:
        logging.error(f"FATAL ERROR | \n{e}")
        exit()

    logger = logging.getLogger()
    logger.info("DEBUT DE MISSION")

    try:
        config_json_path = "config.json"
        with open(config_json_path, "r", encoding="utf-8") as f:
            config = json.load(f)
    except Exception as e:
        logger.error(f"FATAL ERROR | \n{e}")
        exit()

    current_map = topography.Map(config)

    state = change_drone_state(logger, None, 11)
    start_time = time.monotonic()
    lat, lon = 0, 0
    altitude = 0 # mis à jour en temps réel par le gps_thread (Pixhawk)

    thread_ctx = threads.start_background_threads(logger, config)

    try:
        main_loop(
            current_map=current_map,
            state=state,
            start_time=start_time,
            lat=lat,
            lon=lon,
            altitude=altitude,
            logger=logger,
            thread_ctx=thread_ctx,
            config=config,
        )
    finally:
        threads.stop_background_threads(thread_ctx)


def main_loop(current_map, state, start_time, lat, lon, altitude, logger, thread_ctx, config):
    """
    La boucle principale du drone

    étapes pour un état normal : 
            # récupération des données

            # traitement des données

            # Réaction et planification

    différents états :
    10 : fin de mission
    11 : au sol
    12 : décollage
    13 : atterrissage
    14 : en vol
    15 : scan local
    ...
    40 : erreur indéterminée
    ...
    71 : retour d'urgence
    72 : attétrissage d'urgence

    """

    current_path = []
    latest_gps = None
    last_ping_time = 0
    ping_period = 120

    running = True

    stop_event = thread_ctx["stop_event"]
    health = thread_ctx["health"]
    rx_queue = thread_ctx["rx_queue"]
    send_queue = thread_ctx["send_queue"]
    gps_queue = thread_ctx["gps_queue"]
    
    alert_queue = thread_ctx["alert_queue"]

    health.register("main")

    #variable temporaire pour les tests
    loops = 0

    while running and not stop_event.is_set():
        loop_start = time.monotonic()

        try:
            try:
                alert = alert_queue.get_nowait()
                raise RuntimeError(f"watchdog alert: {alert}")
            except queue.Empty:
                pass

            while not rx_queue.empty():
                msg = rx_queue.get()
                logger.info(f"SMS RECU | {msg}")

            while not gps_queue.empty():
                latest_gps = gps_queue.get()
                logger.info(f"GPS RECU | {latest_gps}")

                if "lat" in latest_gps:
                    lat = latest_gps["lat"]

                if "lon" in latest_gps:
                    lon = latest_gps["lon"]

                if "altitude" in latest_gps:
                    altitude = latest_gps["altitude"]

            now = time.monotonic()

            if now - last_ping_time >= ping_period:
                battery = 100 - loops  # valeur temporaire tant que la batterie réelle n'est pas lue

                ping = build_state_ping(
                    config=config,
                    state=state,
                    start_time=start_time,
                    lat=lat,
                    lon=lon,
                    altitude=altitude,
                    battery=battery,
                    current_path=current_path,
                    latest_gps=latest_gps
                )

                send_queue.put(ping)
                last_ping_time = now

                logger.info(f"PING AJOUTE A LA FILE D'ENVOI | {ping}")

            match state:
                case 11:
                    log_state(logger, loops, loops, loops, 100-loops)
                    
                    if loops == 100:running = False
                    else : loops += 1

                    # au sol
                    pass
                case 12:
                    # decollage
                    pass
                case 13:
                    # atterrissage
                    pass
                case 14:
                    # en vol
                    pass
                case 15:
                    # scan local
                    pass

                case 40:
                    # urgence indéterminée
                    pass
                case 41:
                    # retour d'urgence
                    pass

                case _:
                    print("Etat inconnu")
                    logger.warning(f"Etat inconnu | {state}")

            health.progress("main")

        except Exception as e:
            logger.critical(f"loop crash: {e}", exc_info=True)
            #emergency_procedure()
            #reset_system_state()
            state = change_drone_state(logger, state, 40)

        elapsed = time.monotonic() - loop_start
        target_period = 0.5
        if elapsed < target_period:
            time.sleep(target_period - elapsed)

    logger.info("FIN DE MISSION")


if __name__ == "__main__":
    init_drone()