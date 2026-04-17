import heapq
import math
import numpy as np

EPSILON_ALTITUDE = 2   # gardé pour compatibilité (non utilisé dans le nouveau lissage)

# ── Paramètres de lissage (à ajuster pour la soutenance) ──────────────────
RDP_TOLERANCE   = 1      # mètres — tolérance RDP 3D : plus grand = plus lissé
MAX_CLIMB_ANGLE = 15.0   # degrés — angle de montée/descente max autorisé
# ──────────────────────────────────────────────────────────────────────────


def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_cost(current_map, current_node, neighbor_node, penalty, wind=None, energy_mdl=None):
    """
    Calcule le coût de déplacement entre deux nœuds de la grille.

    Sans wind/energy_mdl  → coût = distance euclidienne + pénalité altitude  (comportement original)
    Avec wind + energy_mdl → coût = énergie du segment (kJ)  + pénalité altitude
                              L'énergie intègre : puissance moteur × temps de vol,
                              lui-même dépendant de la composante de vent de face/arrière.

    Paramètres
    ----------
    current_node  : (row, col) nœud courant dans la grille raster
    neighbor_node : (row, col) nœud voisin
    penalty       : float — coefficient de pénalité d'altitude (0 = ignorée)
    wind          : dict  — {'u': float, 'v': float, ...} composantes vent en m/s
                            (u = Est, v = Nord) — None pour désactiver
    energy_mdl    : DroneEnergyModel — None pour désactiver
    """
    p1 = current_map.dataset.xy(current_node[0],  current_node[1])
    p2 = current_map.dataset.xy(neighbor_node[0], neighbor_node[1])

    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    if energy_mdl is not None and wind is not None:
        headwind = energy_mdl.project_wind_on_segment(p1, p2, wind)
        energy_j, _ = energy_mdl.segment_energy_j(dist, {"headwind": headwind})
        base_cost = energy_j / 1000.0
    else:
        base_cost = dist

    elevation = current_map.raster_data[neighbor_node[0], neighbor_node[1]]
    elevation_penalty = elevation * penalty

    return base_cost + elevation_penalty


def astar(current_map, start_lon_lat, end_lon_lat, penalty, wind=None, energy_mdl=None):
    """
    Exécute l'algorithme A* pour trouver le chemin optimal entre deux points GPS.

    Paramètres
    ----------
    start_lon_lat : (lon, lat) ou (x_proj, y_proj) point de départ
    end_lon_lat   : (lon, lat) ou (x_proj, y_proj) point d'arrivée
    penalty       : float — pénalité d'altitude (0 = chemin le plus court/économique pur)
    wind          : dict  — vecteur vent {'u', 'v', 'speed', 'dir_deg'} ou None
    energy_mdl    : DroneEnergyModel ou None

    Retourne
    --------
    list de (x_proj, y_proj) — chemin lissé, ou None si aucun chemin trouvé.
    """
    start_proj = current_map.convert_coords(*start_lon_lat)
    end_proj   = current_map.convert_coords(*end_lon_lat)

    start_node = current_map.dataset.index(*start_proj)
    end_node   = current_map.dataset.index(*end_proj)

    if not current_map.is_valid(*start_node) or not current_map.is_valid(*end_node):
        print("Erreur : Le départ ou l'arrivée se situe en zone interdite (altitude ou hors zone).")
        return None

    open_set = []
    heapq.heappush(open_set, (0, start_node))

    came_from = {}
    g_score   = {start_node: 0}
    f_score   = {start_node: heuristic(start_node, end_node)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end_node:
            raw_path = reconstruct_path(current_map, came_from, current)
            return smooth_path_by_elevation(current_map, raw_path, epsilon=EPSILON_ALTITUDE)

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            neighbor = (current[0] + dr, current[1] + dc)

            if current_map.is_valid(neighbor[0], neighbor[1]):
                tentative_g = g_score[current] + get_cost(
                    current_map, current, neighbor, penalty,
                    wind=wind, energy_mdl=energy_mdl
                )

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f_score[neighbor]   = tentative_g + heuristic(neighbor, end_node)
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
    Retourne la liste des indices à conserver.
    """
    if len(points_3d) <= 2:
        return list(range(len(points_3d)))

    keep    = [False] * len(points_3d)
    keep[0] = True
    keep[-1] = True

    stack = [(0, len(points_3d) - 1)]
    while stack:
        start, end = stack.pop()
        if end - start <= 1:
            continue

        p1      = np.array(points_3d[start], dtype=float)
        p2      = np.array(points_3d[end],   dtype=float)
        segment = p2 - p1
        seg_len = np.linalg.norm(segment)

        if seg_len == 0:
            continue

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
    Si l'angle dépasse max_angle_deg, insère le point qui réduit le mieux la pente.
    """
    max_angle_rad = np.radians(max_angle_deg)
    result  = list(indices)
    changed = True

    while changed:
        changed    = False
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

            dh    = abs(p2[2] - p1[2])
            angle = np.arctan2(dh, horiz)

            if angle > max_angle_rad:
                best_idx   = None
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
                              epsilon=None,
                              max_step=None,
                              rdp_tol=None,
                              max_climb=None):
    """
    Lissage 3D du chemin en deux étapes :
      1. RDP 3D  — supprime les micro-variations (zigzags)
      2. Contrôle d'angle — insère des waypoints si la pente est trop raide
    """
    if len(path) <= 2:
        return path

    tol   = rdp_tol   if rdp_tol   is not None else RDP_TOLERANCE
    angle = max_climb if max_climb is not None else MAX_CLIMB_ANGLE

    pts3d = []
    for pt in path:
        h = current_map.get_fly_height(pt[0], pt[1])
        pts3d.append((pt[0], pt[1], h))

    kept_indices = _rdp_3d(pts3d, tol)
    kept_indices = _split_by_climb_angle(pts3d, kept_indices, angle)

    return [path[i] for i in kept_indices]


# ════════════════════════════════════════════════════════════════════════════
#  Gestion des priorités
# ════════════════════════════════════════════════════════════════════════════

def _normalize_targets(targets):
    """
    Normalise la liste de targets pour accepter deux formats :
      - ancien format : liste de tuples  (lon, lat)
      - nouveau format : liste de dicts  {"point": (lon, lat), "priority": int}

    Retourne une liste de dicts normalisés.
    Priority 0 = urgence maximale (visité en premier).
    """
    normalized = []
    for t in targets:
        if isinstance(t, dict):
            normalized.append({
                "point":    t["point"],
                "priority": int(t.get("priority", 1)),
            })
        else:
            # tuple / liste → priorité neutre par défaut
            normalized.append({"point": t, "priority": 1})
    return normalized


def _greedy_order_within_priority(current_map, start_pos, group):
    """
    Ordonne un groupe de targets (même priorité) par nearest-neighbor greedy
    à partir de start_pos.  Retourne la liste ordonnée des dicts.
    """
    remaining = group.copy()
    ordered   = []
    pos       = start_pos

    while remaining:
        next_t = min(
            remaining,
            key=lambda t: heuristic(
                current_map.convert_coords(*pos),
                current_map.convert_coords(*t["point"]),
            ),
        )
        ordered.append(next_t)
        pos = next_t["point"]
        remaining.remove(next_t)

    return ordered


def _build_ordered_targets(current_map, start_pos, targets_norm):
    """
    Construit la séquence complète de visite :
      1. Regroupe les targets par niveau de priorité.
      2. Traite les groupes du niveau le plus urgent (0) au moins urgent.
      3. Au sein de chaque groupe, applique le greedy nearest-neighbor.

    Retourne une liste ordonnée de dicts {"point", "priority"}.
    """
    from itertools import groupby

    # Tri stable par priorité croissante
    sorted_targets = sorted(targets_norm, key=lambda t: t["priority"])

    ordered_all = []
    current_pos = start_pos

    for _priority, group_iter in groupby(sorted_targets, key=lambda t: t["priority"]):
        group = list(group_iter)
        ordered_group = _greedy_order_within_priority(current_map, current_pos, group)
        ordered_all.extend(ordered_group)
        if ordered_group:
            current_pos = ordered_group[-1]["point"]

    return ordered_all


# ════════════════════════════════════════════════════════════════════════════
#  Boucle principale
# ════════════════════════════════════════════════════════════════════════════

def boucle_principale(current_map, start_node, targets, penalty,
                      fixed_end=None,
                      wind=None, energy_mdl=None):
    """
    Planifie le trajet aller (multi-cibles, avec priorités) puis le retour.

    Paramètres
    ----------
    start_node : (lon, lat) — point de départ/retour
    targets    : liste de tuples  (lon, lat)                        ← ancien format (compatible)
              OU liste de dicts   {"point": (lon, lat),             ← nouveau format
                                   "priority": int}
                 priority 0 = urgence maximale (visité en premier).
                 Les targets de même priorité sont ordonnés greedy entre eux.

    fixed_end  : None | (lon, lat)
                 • None      → le point final de l'aller est choisi automatiquement
                               parmi tous les targets pour minimiser le trajet de retour
                               vers start_node.  Le retour est alors optimisé.
                 • (lon,lat) → le drone doit terminer l'aller sur ce point précis
                               (doit être présent dans targets).

    wind       : dict vent (optionnel) transmis à astar → get_cost
    energy_mdl : DroneEnergyModel (optionnel) transmis à astar → get_cost

    Retourne
    --------
    aller_path  : list de (x_proj, y_proj)
    retour_path : list de (x_proj, y_proj)
    """

    # ── Normalisation ─────────────────────────────────────────────────────
    targets_norm = _normalize_targets(targets)

    if not targets_norm:
        return [], []

    # ── Cas fixed_end : on retire le point final de la liste libre,
    #    on planifie les autres d'abord, puis on force la visite du fixed_end ──
    if fixed_end is not None:
        # Cherche le target correspondant à fixed_end (comparaison lâche)
        end_candidates = [t for t in targets_norm
                          if t["point"] == fixed_end or t["point"] == tuple(fixed_end)]
        free_targets   = [t for t in targets_norm
                          if t not in end_candidates]

        ordered = _build_ordered_targets(current_map, start_node, free_targets)
        # Ajoute le(s) fixed_end en dernier (priorité forcée)
        ordered += end_candidates
        print(f"[Planification] Ordre de visite (fixed_end={fixed_end}) :")

    else:
        # ── Mode optimisation de fin : on planifie dans l'ordre priorité/greedy,
        #    MAIS on réordonne le dernier groupe libre pour minimiser le retour ──
        ordered = _build_ordered_targets(current_map, start_node, targets_norm)

        # Optimisation du dernier tronçon : parmi les targets du groupe de
        # priorité la plus basse, choisir lequel placer en dernier pour
        # minimiser dist(dernier → start_node).
        if len(ordered) >= 2:
            last_priority = ordered[-1]["priority"]
            # Sépare le dernier groupe libre
            last_group_start = next(
                (i for i in range(len(ordered) - 1, -1, -1)
                 if ordered[i]["priority"] != last_priority),
                -1
            ) + 1
            last_group = ordered[last_group_start:]
            prefix     = ordered[:last_group_start]

            if len(last_group) > 1:
                # Point de départ du dernier groupe
                pre_pos = prefix[-1]["point"] if prefix else start_node

                # Cherche le target du groupe qui minimise dist(target → start_node)
                best_end = min(
                    last_group,
                    key=lambda t: heuristic(
                        current_map.convert_coords(*t["point"]),
                        current_map.convert_coords(*start_node),
                    ),
                )
                # Réordonne le dernier groupe : greedy depuis pre_pos,
                # mais force best_end en dernière position
                last_group_without_best = [t for t in last_group if t is not best_end]
                reordered_last = _greedy_order_within_priority(
                    current_map, pre_pos, last_group_without_best
                )
                reordered_last.append(best_end)
                ordered = prefix + reordered_last

        print(f"[Planification] Ordre de visite optimisé (dernier = plus proche du retour) :")

    for i, t in enumerate(ordered):
        prio_str = f"priorité {t['priority']}" if t['priority'] != 1 else "priorité normale"
        print(f"  [{i+1}] {t['point']}  ({prio_str})")

    # ── Construction du chemin aller ──────────────────────────────────────
    aller_path  = []
    current_pos = start_node

    for t in ordered:
        target_pt = t["point"]
        segment = astar(current_map, current_pos, target_pt, penalty,
                        wind=wind, energy_mdl=energy_mdl)

        if segment:
            segment = smooth_path_by_elevation(current_map, segment, epsilon=EPSILON_ALTITUDE)
            if not aller_path:
                aller_path.extend(segment)
            else:
                aller_path.extend(segment[1:])
            current_pos = target_pt
        else:
            print(f"  ⚠ Aucun chemin vers {target_pt}, cible ignorée.")

    # ── Phase retour ──────────────────────────────────────────────────────
    print(f"[Planification] Retour : {current_pos} → {start_node}")
    retour_path = []

    if current_pos == start_node:
        print("  Déjà au point de départ.")
    else:
        segment_retour = astar(current_map, current_pos, start_node, penalty,
                               wind=wind, energy_mdl=energy_mdl)
        if segment_retour and len(segment_retour) >= 2:
            retour_path = smooth_path_by_elevation(
                current_map, segment_retour, epsilon=EPSILON_ALTITUDE
            )
        else:
            retour_path = []

    return aller_path, retour_path