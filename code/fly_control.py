import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, current_map):
        """
        Initialise le planificateur A* avec les données de la carte et les contraintes.
        """
        self.map = current_map
        # Récupération de la matrice des données raster (altitudes)
        self.grid = self.map.raster_data
        self.rows, self.cols = self.grid.shape
        
        # Paramètres de contraintes issus de la configuration
        self.max_tree_height = self.map.max_tree_height
        self.security_height = self.map.security_height
        self.max_fly_height = self.map.max_fly_height

    def heuristic(self, a, b):
        """
        Calcule la distance euclidienne entre deux points (Estimation heuristique).
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def is_valid(self, row, col):
        """
        Vérifie si une cellule (pixel) est accessible pour le drone.
        """
        # 1. Vérifier si les indices sont dans les limites de la matrice
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        
        # 2. Vérifier la contrainte d'altitude maximale
        # Altitude totale = Terrain + Hauteur arbres + Marge de sécurité
        terrain_height = self.grid[row, col]
        required_height = terrain_height + self.max_tree_height + self.security_height
        
        if required_height > self.max_fly_height:
            return False  # Zone trop haute, considérée comme obstacle

        # 3. Vérifier si le point est dans le polygone de mission défini dans config.json
        x, y = self.map.dataset.xy(row, col)
        return self.map.in_map_shape(x, y)

    def get_cost(self, current_node, neighbor_node):
        """
        Calcule le coût de déplacement : Coût de distance + Pénalité d'altitude.
        """
        # Coût de distance de base (1.0 pour orthogonal, 1.414 pour diagonal)
        dist = np.sqrt((current_node[0] - neighbor_node[0])**2 + (current_node[1] - neighbor_node[1])**2)
        
        # Pénalité d'altitude : Encourage le drone à rester dans les zones basses
        elevation = self.grid[neighbor_node[0], neighbor_node[1]]
        elevation_penalty = elevation * 0.5 
        
        return dist + elevation_penalty

    def plan(self, start_lon_lat, end_lon_lat):
        """
        Exécute l'algorithme A* pour trouver le chemin optimal entre deux points GPS.
        """
        # Conversion : (Lon, Lat) -> Coordonnées projetées -> Indices de matrice (row, col)
        start_proj = self.map.convert_coords(*start_lon_lat)
        end_proj = self.map.convert_coords(*end_lon_lat)
        
        start_node = self.map.dataset.index(*start_proj)
        end_node = self.map.dataset.index(*end_proj)

        # Vérification initiale de validité du départ et de l'arrivée
        if not self.is_valid(*start_node) or not self.is_valid(*end_node):
            print("Erreur : Le départ ou l'arrivée se situe en zone interdite (altitude ou hors zone).")
            return None

        # Initialisation de la file de priorité (Open Set)
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        came_from = {} # Pour reconstruire le chemin
        g_score = {start_node: 0} # Coût du départ au nœud actuel
        f_score = {start_node: self.heuristic(start_node, end_node)} # Estimation totale

        while open_set:
            # Récupérer le nœud avec le f_score le plus bas
            current = heapq.heappop(open_set)[1]

            # Vérifier si l'objectif est atteint
            if current == end_node:
                return self.reconstruct_path(came_from, current)

            # Exploration des 8 voisins (horizontaux, verticaux et diagonaux)
            for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dr, current[1] + dc)

                if self.is_valid(neighbor[0], neighbor[1]):
                    tentative_g_score = g_score[current] + self.get_cost(current, neighbor)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        # Ce chemin est le meilleur trouvé jusqu'à présent
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end_node)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None # Aucun chemin trouvé

    def reconstruct_path(self, came_from, current):
        """
        Reconstruit le chemin du but vers le départ et le convertit en coordonnées GPS.
        """
        path = []
        while current in came_from:
            # Convertir les indices de matrice en (Lon, Lat)
            lon_lat = self.map.dataset.xy(current[0], current[1])
            path.append(lon_lat)
            current = came_from[current]
        
        # Inverser pour avoir le chemin du départ vers l'arrivée
        return path[::-1]