import heapq
import numpy as np

EPSILON_ALTITUDE = 2   # gardé pour compatibilité (non utilisé dans le nouveau lissage)

# ── Paramètres de lissage (à ajuster pour la soutenance) ──────────────────
RDP_TOLERANCE   = 5    # mètres — tolérance RDP 3D : plus grand = plus lissé
MAX_CLIMB_ANGLE = 15.0   # degrés — angle de montée/descente max autorisé
# ──────────────────────────────────────────────────────────────────────────

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
    
    came_from = {}
    g_score = {start_node: 0}
    f_score = {start_node: heuristic(start_node, end_node)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end_node:
            raw_path = reconstruct_path(current_map, came_from, current)
            return smooth_path_by_elevation(current_map, raw_path, epsilon=EPSILON_ALTITUDE)

        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            neighbor = (current[0] + dr, current[1] + dc)

            if current_map.is_valid(neighbor[0], neighbor[1]):
                tentative_g_score = g_score[current] + get_cost(current_map, current, neighbor, penalty)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_node)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

def reconstruct_path(current_map, came_from, current):
    path = []
    while current in came_from:
        lon_lat = current_map.dataset.xy(current[0], current[1])
        path.append(lon_lat)
        current = came_from[current]
    
    lon_lat = current_map.dataset.xy(current[0], current[1])
    path.append(lon_lat)
    
    return path[::-1]

def _rdp_3d(points_3d, tolerance):
    """
    Ramer-Douglas-Peucker sur des points 3D (x, y, altitude).
    Retourne un masque booléen des indices à conserver.
    """
    if len(points_3d) <= 2:
        return list(range(len(points_3d)))

    keep = [False] * len(points_3d)
    keep[0] = True
    keep[-1] = True

    stack = [(0, len(points_3d) - 1)]
    while stack:
        start, end = stack.pop()
        if end - start <= 1:
            continue

        p1 = np.array(points_3d[start], dtype=float)
        p2 = np.array(points_3d[end],   dtype=float)
        segment = p2 - p1
        seg_len = np.linalg.norm(segment)

        if seg_len == 0:
            continue

        # Distance perpendiculaire de chaque point intermédiaire au segment p1-p2
        max_dist = 0.0
        max_idx  = start
        for i in range(start + 1, end):
            pt   = np.array(points_3d[i], dtype=float)
            proj = np.dot(pt - p1, segment) / (seg_len ** 2)
            proj = max(0.0, min(1.0, proj))
            closest = p1 + proj * segment
            dist = np.linalg.norm(pt - closest)
            if dist > max_dist:
                max_dist = dist
                max_idx  = i

        if max_dist > tolerance:
            keep[max_idx] = True
            stack.append((start, max_idx))
            stack.append((max_idx, end))

    return [i for i, k in enumerate(keep) if k]


def _split_by_climb_angle(path_3d, indices, max_angle_deg):
    """
    Après RDP, vérifie l'angle de montée/descente entre waypoints consécutifs.
    Si l'angle dépasse max_angle_deg, insère le point le plus haut du segment intermédiaire.
    path_3d : liste de (x, y, altitude) — coordonnées complètes avant RDP.
    indices  : indices retenus par RDP (dans path_3d).
    """
    max_angle_rad = np.radians(max_angle_deg)
    result = list(indices)
    changed = True

    while changed:
        changed = False
        new_result = [result[0]]
        for k in range(1, len(result)):
            i_prev = result[k - 1]
            i_curr = result[k]
            p1 = np.array(path_3d[i_prev])
            p2 = np.array(path_3d[i_curr])

            horiz = np.linalg.norm(p2[:2] - p1[:2])
            if horiz < 1e-6:
                new_result.append(i_curr)
                continue

            dh = abs(p2[2] - p1[2])
            angle = np.arctan2(dh, horiz)

            if angle > max_angle_rad:
                # Cherche le point intermédiaire qui réduit le mieux l'angle
                best_idx = None
                best_angle = angle
                for j in range(i_prev + 1, i_curr):
                    pm = np.array(path_3d[j])
                    h1 = np.linalg.norm(pm[:2] - p1[:2])
                    h2 = np.linalg.norm(p2[:2] - pm[:2])
                    if h1 < 1e-6 or h2 < 1e-6:
                        continue
                    a1 = np.arctan2(abs(pm[2] - p1[2]), h1)
                    a2 = np.arctan2(abs(p2[2] - pm[2]), h2)
                    worst = max(a1, a2)
                    if worst < best_angle:
                        best_angle = worst
                        best_idx   = j

                if best_idx is not None:
                    new_result.append(best_idx)
                    changed = True

            new_result.append(i_curr)
        result = new_result

    return result


def smooth_path_by_elevation(current_map, path,
                              epsilon=None,        # ignoré, gardé pour compatibilité
                              max_step=None,       # ignoré
                              rdp_tol=None,
                              max_climb=None):
    """
    Lissage 3D du chemin en deux étapes :
      1. RDP 3D  — supprime les micro-variations (zigzags)
      2. Contrôle d'angle — insère des waypoints si la pente est trop raide

    Paramètres (priorité : argument > constante globale) :
      rdp_tol   : tolérance RDP en mètres        (défaut : RDP_TOLERANCE)
      max_climb : angle de montée max en degrés   (défaut : MAX_CLIMB_ANGLE)
    """
    if len(path) <= 2:
        return path

    tol   = rdp_tol   if rdp_tol   is not None else RDP_TOLERANCE
    angle = max_climb if max_climb is not None else MAX_CLIMB_ANGLE

    # Construction des points 3D : (x_proj, y_proj, altitude_vol)
    pts3d = []
    for pt in path:
        h = current_map.get_fly_height(pt[0], pt[1])
        pts3d.append((pt[0], pt[1], h))

    # Étape 1 : RDP 3D
    kept_indices = _rdp_3d(pts3d, tol)

    # Étape 2 : contrôle angle de montée
    kept_indices = _split_by_climb_angle(pts3d, kept_indices, angle)

    return [path[i] for i in kept_indices]

def boucle_principale(current_map, start_node, targets, penalty):
    aller_path = [] 
    retour_path = [] 
    current_pos = start_node
    remaining_targets = targets.copy()

    while remaining_targets:
        next_target = min(remaining_targets, 
                          key=lambda t: heuristic(
                              current_map.convert_coords(*current_pos), 
                              current_map.convert_coords(*t)
                          ))
        
        # Utiliser astar pour obtenir segment -> smoothing
        segment = astar(current_map, current_pos, next_target, penalty)

        if segment:
            segment = smooth_path_by_elevation(current_map, segment, epsilon=EPSILON_ALTITUDE)
            
            if not aller_path:
                aller_path.extend(segment)
            else:
                aller_path.extend(segment[1:])
            
            current_pos = next_target
            remaining_targets.remove(next_target)
        else:
            remaining_targets.remove(next_target)

    # PHASE RETOUR
    print(f"Planification du retour : {current_pos} -> {start_node}")
    segment_retour = astar(current_map, current_pos, start_node, penalty)
    
    if segment_retour:
        retour_path = smooth_path_by_elevation(current_map, segment_retour, epsilon=EPSILON_ALTITUDE)
    else:
        retour_path = []

    return aller_path, retour_path