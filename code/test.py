import graphics, random, fly_control
import numpy as np
import random

def polygon_random_points (current_map):
    min_x, min_y, max_x, max_y = min(current_map.map_shape, key=lambda x: x[0])[0], min(current_map.map_shape, key=lambda x: x[1])[1], max(current_map.map_shape, key=lambda x: x[0])[0], max(current_map.map_shape, key=lambda x: x[1])[1]
    while True:
        random_point = (random.uniform(min_x, max_x), random.uniform(min_y, max_y))
        if current_map.in_map_shape(*random_point):
            return random_point

def generic_test(current_map):

    graphics.map_mutli_points_plot(current_map.dataset, current_map.map_shape)
    """
    méthodes d'affihage

    print(current_map.get_elevation(3451535.0,2752425.0))
    print(current_map.map_shape)
    print(current_map.in_map_shape(3451535.0,2752425.0))

    print(current_map.__str__())

    graphics.height_plot(current_map.raster_data[5], current_map.security_height, current_map.max_tree_height, current_map.max_fly_height)
    """


def carte_et_points(current_map, nb_points):
    bounding_box = current_map.dataset.bounds

    points_in_shape = []
    points_out_shape = []

    for i in range(nb_points):
        x = random.uniform(bounding_box.left, bounding_box.right)
        y = random.uniform(bounding_box.bottom, bounding_box.top)
        if current_map.in_map_shape(x, y):
            points_in_shape.append((x, y))
        else:
            points_out_shape.append((x, y))

    graphics.map_mutli_points_plot(current_map.dataset, current_map.map_shape, points_in_shape, points_out_shape)

def astar(current_map, nb_paths):
    for i in range(nb_paths):
        start_pt = polygon_random_points(current_map)
        end_pt = polygon_random_points(current_map)

        start_proj = current_map.convert_coords(*start_pt)
        end_proj = current_map.convert_coords(*end_pt)

        print(f"Coordonnées projetées Départ : {start_proj}, Dans la zone : {current_map.in_map_shape(*start_proj)}")
        print(f"Coordonnées projetées Arrivée : {end_proj}, Dans la zone : {current_map.in_map_shape(*end_proj)}")

        # Calcul de la trajectoire optimale via l'algorithme A*
        path = fly_control.astar(current_map, start_pt, end_pt)

        if path:
            print(f"Succès : Chemin trouvé avec {len(path)} waypoints.")
            
            graphics.map_mutli_points_plot(
                dataset=current_map.dataset, 
                map_shape=current_map.map_shape, 
                points_in_shape=path,
                points_out_shape=[start_pt, end_pt],
                draw_path=True
            )

            print(path)

            fly_path = np.array([current_map.get_fly_height(*pt) for pt in path])

            n = 10
            heigth_array = []
            for k in range(len(path) - 1):
                x = np.linspace(path[k][0], path[k + 1][0], n, endpoint=False)
                y = np.linspace(path[k][1], path[k + 1][1], n, endpoint=False)
                for j in range(n):
                    heigth_array.append(current_map.get_elevation(x[j], y[j]))

            heigth_array.append(current_map.get_elevation(path[-1][0], path[-1][1]))
            heigth_array = np.array(heigth_array)

            fly_array_x = np.arange(len(path)) * n

            graphics.height_plot(
                heigth_array,
                current_map.security_height,
                current_map.max_tree_height,
                current_map.max_fly_height,
                fly_array_y=fly_path,
                fly_array_x=fly_array_x
            )
        else:
            print("Erreur : Aucun chemin valide trouvé. Vérifiez si les points sont bien dans la zone autorisée.")


def astar_comparaison(current_map, n = 1):
    paths = []
    titles = []
    start_pt = (-1.531098974090213,
            47.28897545723686)
    end_pt = (-1.5146818725710602,
            47.284347803564884)

    start_proj = current_map.convert_coords(*start_pt)
    end_proj = current_map.convert_coords(*end_pt)

    penaltys = np.linspace(0, 1, n)

    for i in range(n):

        print(f"Coordonnées projetées Départ : {start_proj}, Dans la zone : {current_map.in_map_shape(*start_proj)}")
        print(f"Coordonnées projetées Arrivée : {end_proj}, Dans la zone : {current_map.in_map_shape(*end_proj)}")

        # Calcul de la trajectoire optimale via l'algorithme A*
        path = fly_control.astar(current_map, start_pt, end_pt, penaltys[i])

        if path:
            print(f"Succès : Chemin trouvé avec {len(path)} waypoints.")
            paths.append(path)
            titles.append(f"Penalty : {penaltys[i]}")
        else:
            print("Erreur : Aucun chemin valide rencontré. Vérifiez si les points sont bien dans la zone autorisée.")
            paths.append([])
            titles.append(f"Penalty : {penaltys[i]}")
    
    print(len(paths))
    graphics.mutli_points_plot_path(current_map, paths)
    graphics.mutliplot_path(current_map, paths, titles)
    
def main_temp():
  config = json.load(open("config.json"))
    current_map = topography.Map(config)

    try:
        print("Génération des points de mission...")
        point_A = current_map.get_random_valid_point() 
        points_initiales = [current_map.get_random_valid_point() for _ in range(3)] 
        
        print("\n--- 1. Calcul du trajet INITIAL ---")
        path_aller_init, path_retour_init = fly_control.boucle_principale(
            current_map, point_A, points_initiales, penalty=0.5
        )
        
        data_avant = {
            'path_aller': path_aller_init,
            'path_retour': path_retour_init,
            'points_out': [point_A] + points_initiales
        }

        print("\n--- 2. Simulation DYNAMIQUE ---")
        target_B = points_initiales[0]
        try:
            idx_at_B = path_aller_init.index(target_B)
            drone_current_pos = path_aller_init[idx_at_B]
        except ValueError:
            drone_current_pos = target_B
            idx_at_B = 0 

        print(f"Le drone est officiellement au point B : {drone_current_pos}")

        point_NEW = current_map.get_random_valid_point()
        print(f"Saisie d'un nouveau point NEW: {point_NEW}")

        remaining_targets = points_initiales[1:] + [point_NEW]
        
        path_aller_dyn_new_segments = []
        temp_pos = drone_current_pos
        targets_to_visit = remaining_targets.copy()
        
        while targets_to_visit:
            next_target = min(targets_to_visit, key=lambda t: fly_control.heuristic(
                current_map.convert_coords(*temp_pos), 
                current_map.convert_coords(*t)
            ))
            segment = fly_control.astar(current_map, temp_pos, next_target, 0.5)
            if segment:
                if not path_aller_dyn_new_segments: 
                    path_aller_dyn_new_segments.extend(segment)
                else: 
                    path_aller_dyn_new_segments.extend(segment[1:])
                temp_pos = next_target
                targets_to_visit.remove(next_target)
            else: 
                targets_to_visit.remove(next_target)

        path_retour_dyn = fly_control.astar(current_map, temp_pos, point_A, 0.5)

        path_total_dyn = path_aller_init[:idx_at_B+1] + path_aller_dyn_new_segments[1:]
        
        data_apres = {
            'path_aller': path_total_dyn,
            'path_retour': path_retour_dyn,
            'points_out': [drone_current_pos, point_A] + remaining_targets
        }

        print("\nAffichage du comparatif (Retour en NOIR)...")
        graphics.map_mutli_points_plot_compare(
            dataset=current_map.dataset, 
            map_shape=current_map.map_shape, 
            data_before=data_avant,
            data_after=data_apres
        )
        test.astar_comparaison(current_map, n = 3)
            
    except Exception as e:
        print(f"Erreur lors de la mission : {e}")
        import traceback
        traceback.print_exc()