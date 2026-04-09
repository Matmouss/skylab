import heapq
import numpy as np

EPSILON_ALTITUDE = 0.1

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
    
    # 加入起点
    lon_lat = current_map.dataset.xy(current[0], current[1])
    path.append(lon_lat)
    
    return path[::-1]

def smooth_path_by_elevation(current_map, path, epsilon=3.0, max_step=20):
    """
    保留：起点、终点、高度极值点、以及每隔 max_step 个点强制保留一个
    """
    if len(path) <= 2:
        return path

    heights = [current_map.get_fly_height(*pt) for pt in path]
    
    simplified_path = [path[0]]
    last_kept_idx = 0

    for i in range(1, len(path) - 1):
        prev_h = heights[i-1]
        curr_h = heights[i]
        next_h = heights[i+1]

        is_peak = (curr_h > prev_h + epsilon) and (curr_h > next_h + epsilon)
        is_valley = (curr_h < prev_h - epsilon) and (curr_h < next_h - epsilon)
        
        # 强制每隔 max_step 个点保留一个，防止路径退化为直线
        is_forced = (i - last_kept_idx) >= max_step

        if is_peak or is_valley or is_forced:
            simplified_path.append(path[i])
            last_kept_idx = i
            
    simplified_path.append(path[-1])
    
    return simplified_path

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
        
        # ✅ 修复：先调用 astar 获取 segment，再做平滑
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