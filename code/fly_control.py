import heapq
import numpy as np

def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_cost(current_map, current_node, neighbor_node, penalty):
    """
    Calcule le coût de déplacement : Coût de distance + Pénalité d'altitude.
    Coût de distance de base (1.0 pour orthogonal, 1.414 pour diagonal)
    Pénalité d'altitude : Encourage le drone à rester dans les zones basses
    """
    dist = np.sqrt((current_node[0] - neighbor_node[0])**2 + (current_node[1] - neighbor_node[1])**2)
    
    elevation = current_map.  raster_data[neighbor_node[0], neighbor_node[1]]
    elevation_penalty = elevation * penalty
    
    return dist + elevation_penalty

def astar(current_map, start_lon_lat, end_lon_lat, penalty):
    """
    Exécute l'algorithme A* pour trouver le chemin optimal entre deux points GPS.
    """
    start_proj = current_map.convert_coords(*start_lon_lat)
    end_proj = current_map.convert_coords(*end_lon_lat)
    
    start_node = current_map.dataset.index(*start_proj)
    end_node = current_map.dataset.index(*end_proj)

    if not current_map.is_valid(*start_node) or not current_map.is_valid(*end_node):
        print("Erreur : Le départ ou l'arrivée se situe en zone interdite (altitude ou hors zone).")
        return None

    open_set = []
    heapq.heappush(open_set, (0, start_node))
    
    came_from = {} # Pour reconstruire le chemin
    g_score = {start_node: 0} # Coût du départ au nœud actuel
    f_score = {start_node: heuristic(start_node, end_node)} # Estimation totale

    while open_set:
        # Récupérer le nœud avec le f_score le plus bas
        current = heapq.heappop(open_set)[1]

        # Vérifier si l'objectif est atteint
        if current == end_node:
            return reconstruct_path(current_map, came_from, current)

        # Exploration des 8 voisins (horizontaux, verticaux et diagonaux)
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            neighbor = (current[0] + dr, current[1] + dc)

            if current_map.is_valid(neighbor[0], neighbor[1]):
                tentative_g_score = g_score[current] + get_cost(current_map, current, neighbor, penalty)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Ce chemin est le meilleur trouvé jusqu'à présent
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_node)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None # Aucun chemin trouvé

def reconstruct_path(current_map, came_from, current):
    """
    Reconstruit le chemin du but vers le départ et le convertit en coordonnées GPS.
    """
    path = []
    while current in came_from:
        lon_lat = current_map.dataset.xy(current[0], current[1])
        path.append(lon_lat)
        current = came_from[current]
    
    return path[::-1]

def boucle_principale(current_map, start_node, targets, penalty):
    """
    Planifie le trajet multi-points et le retour au départ.
    """
    aller_path = [] 
    retour_path = [] 
    current_pos = start_node
    remaining_targets = targets.copy()

    # --- PHASE ALLER  ---
    while remaining_targets:
        next_target = min(remaining_targets, 
                          key=lambda t: heuristic(
                              current_map.convert_coords(*current_pos), 
                              current_map.convert_coords(*t)
                          ))
        
        segment = astar(current_map, current_pos, next_target, penalty)
        if segment:
            if not aller_path:
                aller_path.extend(segment)
            else:
                aller_path.extend(segment[1:])
            current_pos = next_target
            remaining_targets.remove(next_target)
        else:
            remaining_targets.remove(next_target)

    # --- PHASE RETOUR  ---
    print(f"Planification du retour : {current_pos} -> {start_node}")
    retour_path = astar(current_map, current_pos, start_node, penalty)

    return aller_path, retour_path