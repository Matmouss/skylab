import topography, fly_control, os, json, graphics
import numpy as np

if __name__ == "__main__":
    # Définition du répertoire de travail et chargement de la configuration
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = "config.json"
    config = json.load(open(config_json_path))

    # Initialisation de la carte (topographie) et du planificateur de trajectoire A*
    current_map = topography.Map(config)
    planner = fly_control.AStarPlanner(current_map) 

    # Définition des coordonnées de départ et d'arrivée (Longitude, Latitude)
    start_pt = (-1.5255, 47.2880) 
    end_pt = (-1.524654200951312, 47.2866)

    # Conversion des coordonnées géographiques en coordonnées projetées (système local)
    start_proj = current_map.convert_coords(*start_pt)
    end_proj = current_map.convert_coords(*end_pt)

    # Vérification si les points se situent à l'intérieur du polygone de mission (map_shape)
    print(f"Coordonnées projetées Départ : {start_proj}, Dans la zone : {current_map.in_map_shape(*start_proj)}")
    print(f"Coordonnées projetées Arrivée : {end_proj}, Dans la zone : {current_map.in_map_shape(*end_proj)}")

    # Calcul de la trajectoire optimale via l'algorithme A*
    path = planner.plan(start_pt, end_pt)

    if path:
        print(f"Succès : Chemin trouvé avec {len(path)} waypoints.")
        
        # Visualisation de la carte avec la trajectoire planifiée
        graphics.map_plot(
            dataset=current_map.dataset, 
            map_shape=config["map_shape"], 
            points_in_shape=path,
            points_out_shape=[start_pt, end_pt] 
        )
    else:
        # Message d'erreur si aucun chemin n'est trouvé
        print("Erreur : Aucun chemin valide trouvé. Vérifiez si les points sont bien dans la zone autorisée.")