import topography, fly_control, os, json, graphics, test, time
import numpy as np

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


def main():
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
    40 : urgence indéterminée
    41 : retour d'urgence

    """

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = "config.json"
    config = json.load(open(config_json_path))

    current_map = topography.Map(config)

    state = 11

    running = True

    while running:

        match state:
            case 11:
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





if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = "config.json"
    config = json.load(open(config_json_path))

    current_map = topography.Map(config)

    #test.carte_et_points(current_map, 500)
    #test.astar(current_map,1)
    test.astar_comparaison(current_map, 4)