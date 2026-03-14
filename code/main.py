import topography, fly_control, os, json, graphics, test, time, logging
import numpy as np

def log_state(logger, lat, lon, altitude, battery):
    logger.info(
        f"STATE | lat={lat} lon={lon} alt={altitude}m battery={battery}%"
    )

def change_drone_state(logger, old_state,state):
    logger.info(f"CHANGING DRONE STATE | {old_state} -> {state}")
    return state

def get_data(capteurs = ["camera_rgb", "camera_thermique", "gps", "sms"]):
    data = {}
    if "camera_rgb" in capteurs:
        data["camera_rgb"] = np.random.rand(100, 100, 3)
    if "camera_thermique" in capteurs:
        data["camera_thermique"] = np.random.rand(100, 100)
    if "gps" in capteurs:
        data["gps"] = np.random.rand(2)
    if "sms" in capteurs:
        data["sms"] = np.random.rand(1)

    return data

def send_data(data):
    print(data)

def init_drone():
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)

    try:
        os.remove("../logs/drone.log")
        logging.basicConfig(
            filename=r"..\logs\drone.log",
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(message)s"
        )
    except Exception as e:
        logging.error(f"FATAL ERROR | \n{e}")
        exit()

    logger = logging.getLogger()

    logger.info("DEBUT DE MISSION")

    try :
        config_json_path = "config.json"
        config = json.load(open(config_json_path))
    except Exception as e:
        logger.error(f"FATAL ERROR | \n{e}")
        exit()

    current_map = topography.Map(config)

    state = change_drone_state(logger, None, 11)
    start_time = time.time()
    lat, lon = 0, 0
    altitude = 0 # hors test mentionner l'altitude réelle de départ

def main_loop(current_map, state, start_time, lat, lon, altitude):
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
    running_subprocess = []

    running = True

    while running:
        try:
            match state:
                case 11:
                    for i in range(10):
                        log_state(logger, i, i, i, 100-i)
                    running = False
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

        except Exception as e:
            logger.critical(f"loop crash: {e}", exc_info=True)
            #emergency_procedure()
            #reset_system_state()
            state = change_drone_state(logger, state, 40)

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

    
