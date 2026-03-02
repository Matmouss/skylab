import graphics, random, fly_control
import numpy as np

def generic_test(current_map):


    graphics.map_plot(current_map.dataset, current_map.map_shape)
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

    graphics.map_plot(current_map.dataset, current_map.map_shape, points_in_shape, points_out_shape)

def astar(current_map):
    start_pt = (-1.5255, 47.2880) 
    end_pt = (-1.524654200951312, 47.2866)

    start_proj = current_map.convert_coords(*start_pt)
    end_proj = current_map.convert_coords(*end_pt)

    print(f"Coordonnées projetées Départ : {start_proj}, Dans la zone : {current_map.in_map_shape(*start_proj)}")
    print(f"Coordonnées projetées Arrivée : {end_proj}, Dans la zone : {current_map.in_map_shape(*end_proj)}")

    # Calcul de la trajectoire optimale via l'algorithme A*
    path = fly_control.astar(current_map, start_pt, end_pt)

    if path:
        print(f"Succès : Chemin trouvé avec {len(path)} waypoints.")
        
        graphics.map_plot(
            dataset=current_map.dataset, 
            map_shape=current_map.map_shape, 
            points_in_shape=path,
            points_out_shape=[start_pt, end_pt],
            draw_path=True
        )

        height_array = np.array([current_map.get_elevation(*pt) for pt in path])
        graphics.height_plot(height_array, current_map.security_height, current_map.max_tree_height, current_map.max_fly_height, height_array)
        print(path)
    else:
        # Message d'erreur si aucun chemin n'est trouvé
        print("Erreur : Aucun chemin valide trouvé. Vérifiez si les points sont bien dans la zone autorisée.")