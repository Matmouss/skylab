import topography, fly_control, os, json, graphics, test, time, logging, threads, queue
import numpy as np

def log_state(logger, lat, lon, altitude, battery):
    logger.info(
        f"STATE | lat={lat} lon={lon} alt={altitude}m battery={battery}%"
    )

def change_drone_state(logger, old_state,state):
    logger.info(f"CHANGING DRONE STATE | {old_state} -> {state}")
    return state


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
    altitude = 0 # hors test mentionner l'altitude réelle de départ

    thread_ctx = threads.start_background_threads(logger)

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
        )
    finally:
        threads.stop_background_threads(thread_ctx)


def main_loop(current_map, state, start_time, lat, lon, altitude, logger, thread_ctx):
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
                gps = gps_queue.get()
                logger.info(f"GPS RECU | {gps}")

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
    """
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = "config.json"
    config = json.load(open(config_json_path))



    current_map = topography.Map(config)

    #test.main_temp(current_map)

    #test.carte_et_points(current_map, 500)
    #test.astar(current_map,1)
    test.astar_comparaison(current_map, 4)"""

    init_drone()