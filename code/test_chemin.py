import topography
import fly_control
import graphics
import json
import os
import random
import numpy as np

def get_random_valid_point(current_map):
    """
    Génère un point aléatoire valide à l'intérieur du polygone et respectant les contraintes d'altitude.
    """
    # Récupérer les limites du fichier TIF (Bounding Box)
    bounds = current_map.dataset.bounds
    max_attempts = 2000 # Nombre maximum d'essais pour éviter une boucle infinie
    
    for _ in range(max_attempts):
        # Générer des coordonnées projetées aléatoires
        rand_x = random.uniform(bounds.left, bounds.right)
        rand_y = random.uniform(bounds.bottom, bounds.top)
        
        # 1. Vérifier si le point est à l'intérieur du polygone de mission
        if current_map.in_map_shape(rand_x, rand_y):
            # 2. Vérifier si l'altitude permet le passage du drone
            row, col = current_map.dataset.index(rand_x, rand_y)
            terrain_elevation = current_map.raster_data[row, col]
            
            # Altitude requise = Terrain + Arbres + Marge de sécurité
            required_h = terrain_elevation + current_map.max_tree_height + current_map.security_height
            
            if required_h < current_map.max_fly_height:
                # Convertir les coordonnées projetées en Longitude/Latitude pour l'algorithme
                lon, lat = current_map.dataset.xy(row, col)
                return (lon, lat)
                
    raise Exception("Impossible de trouver un point valide. Vérifiez les contraintes d'altitude ou le polygone.")

def test_random_path():
    """
    Fonction principale pour tester l'algorithme A* avec des points de départ et d'arrivée aléatoires.
    """
    # Définition du répertoire de travail
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    
    # Chargement de la configuration
    config = json.load(open("config.json"))
    
    # Initialisation de la carte et du planificateur
    current_map = topography.Map(config)
    planner = fly_control.AStarPlanner(current_map)
    
    print(f"--- Début du test aléatoire (Altitude Max : {config['max_fly_height']}m) ---")
    
    try:
        # Recherche de deux points valides
        start_pt = get_random_valid_point(current_map)
        end_pt = get_random_valid_point(current_map)
        
        print(f"Point de départ séléctionné : {start_pt}")
        print(f"Point d'arrivée séléctionné : {end_pt}")
        
        # Lancement de la planification de trajectoire
        path = planner.plan(start_pt, end_pt)
        
        if path:
            print(f"Succès ! Chemin trouvé avec {len(path)} waypoints.")
            
            # Visualisation 2D sur la carte
            graphics.map_plot(
                current_map.dataset, 
                config["map_shape"], 
                points_in_shape=path, 
                points_out_shape=[start_pt, end_pt]
            )
            
            # Visualisation du profil altimétrique (Profil de hauteur)
            elevations = [current_map.get_elevation(pt[0], pt[1]) for pt in path]
            graphics.height_plot(
                elevations,
                config["security_height"],
                config["max_tree_height"],
                config["max_fly_height"]
            )
        else:
            print("Échec : L'algorithme n'a pas pu trouver de chemin entre ces points.")
            
    except Exception as e:
        print(f"Erreur lors du test : {e}")

if __name__ == "__main__":
    test_random_path()