import topography, fly_control, os, json, graphics, test
import numpy as np

if __name__ == "__main__":
    # Initialisation de l'environnement
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    
    # Chargement de la config et de la carte
    config = json.load(open("config.json"))
    current_map = topography.Map(config)

    try:
        # Génération automatique de points valides (A à H)
        print("Génération des points de mission...")
        point_A = current_map.get_random_valid_point() # Départ
        points_cibles = [current_map.get_random_valid_point() for _ in range(7)] # B à H
        
        # Planification du trajet complet (Aller + Retour)
        path_aller, path_retour = fly_control.boucle_principale(
            current_map, 
            point_A, 
            points_cibles, 
            penalty=0.5
        )

        if path_aller:
            waypoints_totaux = len(path_aller) + (len(path_retour) if path_retour else 0)
            print(f"Succès : Mission terminée avec {waypoints_totaux} waypoints au total.")
            
            # Affichage de la carte avec les deux trajectoires
            graphics.map_plot(
                dataset=current_map.dataset, 
                map_shape=current_map.map_shape, 
                points_in_shape=path_aller,    
                retour_path=path_retour,       
                points_out_shape=[point_A] + points_cibles, 
                draw_path=True
            )
        else:
            print("Erreur : Aucun trajet n'a pu être planifié.")
            
    except Exception as e:
        print(f"Erreur lors de la mission : {e}")