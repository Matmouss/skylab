import heapq
import numpy as np

def heuristic(a, b):
    """
    Calcule la distance euclidienne entre deux points (Estimation heuristique).
    """
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_cost(current_map, current_node, neighbor_node):
    """
    Calcule le coût de déplacement : Coût de distance + Pénalité d'altitude.
    """
    # Coût de distance de base (1.0 pour orthogonal, 1.414 pour diagonal)
    dist = np.sqrt((current_node[0] - neighbor_node[0])**2 + (current_node[1] - neighbor_node[1])**2)
    
    # Pénalité d'altitude : Encourage le drone à rester dans les zones basses
    elevation = current_map.raster_data[neighbor_node[0], neighbor_node[1]]
    elevation_penalty = elevation * 0.5 
    
    return dist + elevation_penalty

def astar(current_map, start_lon_lat, end_lon_lat):
    """
    Exécute l'algorithme A* pour trouver le chemin optimal entre deux points GPS.
    """
    # Conversion : (Lon, Lat) -> Coordonnées projetées -> Indices de matrice (row, col)
    start_proj = current_map.convert_coords(*start_lon_lat)
    end_proj = current_map.convert_coords(*end_lon_lat)
    
    start_node = current_map.dataset.index(*start_proj)
    end_node = current_map.dataset.index(*end_proj)

    # Vérification initiale de validité du départ et de l'arrivée
    if not current_map.is_valid(*start_node) or not current_map.is_valid(*end_node):
        print("Erreur : Le départ ou l'arrivée se situe en zone interdite (altitude ou hors zone).")
        return None

    # Initialisation de la file de priorité (Open Set)
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
                tentative_g_score = g_score[current] + get_cost(current_map, current, neighbor)

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
        # Convertir les indices de matrice en (Lon, Lat)
        lon_lat = current_map.dataset.xy(current[0], current[1])
        path.append(lon_lat)
        current = came_from[current]
    
    # Inverser pour avoir le chemin du départ vers l'arrivée
    return path[::-1]