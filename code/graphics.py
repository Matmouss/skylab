import shapely
from shapely import geometry
from shapely.geometry import shape, Point, LineString, Polygon
from rasterio.plot import show
import matplotlib 
import matplotlib.pyplot as plt
import numpy as np

def dataset_plot_2(dataset):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    full_img = dataset.read()
    nir = dataset.read(dataset.count)
    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray - Band 1')
    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1')
    plt.show()

def _add_arrows_to_path(ax, pixel_coords, color):

    if len(pixel_coords) < 2:  # 路径太短则不画
        return

    coords = np.array(pixel_coords)
    mid = len(coords) // 2
    r1, c1 = coords[mid]
    r2, c2 = coords[mid + 1]
    
    ax.annotate('', xy=(c2, r2), xytext=(c1, r1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1, mutation_scale=6),
                zorder=4)
def _draw_mission_on_ax(ax, dataset, map_shape, data_dict, title, is_dynamic=False):
    # --- 1. 基础地图绘制 ---
    full_img = dataset.read(1)
    ax.imshow(full_img, cmap='gray')

    # 绘制任务边界
    pixels = [dataset.index(lon, lat) for lon, lat in map_shape]
    ax.plot([p[1] for p in pixels], [p[0] for p in pixels], color='cyan', linewidth=2, label='Limites', zorder=2)

    if not data_dict:
        ax.set_title(title)
        return

    # --- 2. 路径处理与距离计算 ---
    # 我们将去程和回程合并或分别处理。这里以去程 path_aller 为主生成 Profile
    path = data_dict.get('path_aller', [])
    
    if path:
        # 2a. 计算每个 waypoint 的像素坐标（用于左图绘制）
        px_in = [dataset.index(pt[0], pt[1]) for pt in path]
        line_color = '#00FFFF'
        # 在左图画出完整的连续线段，解决你之前“只有点”的问题
        ax.plot([p[1] for p in px_in], [p[0] for p in px_in], color=line_color, 
                 linewidth=1.2, label='Trajet Aller', zorder=3)
        _add_arrows_to_path(ax, px_in, line_color)

        # 2b. 计算物理距离 (x轴)
        # 注意：这里直接使用坐标数值计算。如果你的坐标是经纬度且跨度极大，建议先转投影坐标
        cum_distances = [0]
        total_dist = 0
        for i in range(1, len(path)):
            p1 = np.array(path[i-1])
            p2 = np.array(path[i])
            dist = np.linalg.norm(p1 - p2) # 欧几里得距离
            total_dist += dist
            cum_distances.append(total_dist)
        cum_distances = np.array(cum_distances)

        # --- 3. 准备右图 (Profile View) 的数据 ---
        # 我们需要在两个 Waypoint 之间进行高密度采样，以还原地形起伏
        terrain_interp = []
        fly_interp = []
        x_dist_interp = [] # 存储采样点对应的真实物理距离

        for i in range(len(path) - 1):
            p_start = np.array(path[i])
            p_end = np.array(path[i+1])
            d_start = cum_distances[i]
            d_end = cum_distances[i+1]
            
            # 在这两点之间插值 10 个采样点
            num_samples = 10
            for j in range(num_samples):
                fraction = j / num_samples
                # 线性插值坐标
                interp_pt = p_start + fraction * (p_end - p_start)
                # 线性插值物理距离坐标
                interp_dist = d_start + fraction * (d_end - d_start)
                
                terrain_interp.append(current_map.get_elevation(*interp_pt))
                fly_interp.append(current_map.get_fly_height(*interp_pt))
                x_dist_interp.append(interp_dist)

        # 加入最后一个终点
        terrain_interp.append(current_map.get_elevation(*path[-1]))
        fly_interp.append(current_map.get_fly_height(*path[-1]))
        x_dist_interp.append(cum_distances[-1])

        # 转换为 Numpy 数组方便计算
        terrain_interp = np.array(terrain_interp)
        fly_interp = np.array(fly_interp)
        x_dist_interp = np.array(x_dist_interp)
        
        # --- 4. 绘制右图 (Profile View) ---
        # 注意：此函数通常被外部循环调用，假设 fig.axes[1] 是对应的 Profile Ax
        # 如果你的代码逻辑里 axs 是传入的，请根据实际对象操作。
        # 这里演示如何更新右图：
        import matplotlib.pyplot as plt
        fig = plt.gcf()
        all_axs = fig.get_axes()
        # 寻找当前 ax 对应的右侧绘图区 (假设是成对出现的)
        # 如果这个函数只管左图，请将以下逻辑移至专门画 Profile 的地方
        try:
            # 找到当前 ax 在子图中的索引，假设 profile 在右边
            ax_idx = list(all_axs).index(ax)
            profile_ax = all_axs[ax_idx + 1] 
            
            profile_ax.clear() # 清除旧的索引图
            
            tree_limit   = terrain_interp + current_map.max_tree_height
            safety_limit = tree_limit + current_map.security_height

            profile_ax.fill_between(x_dist_interp, terrain_interp, tree_limit,  color="green",   alpha=0.2, label='Trees')
            profile_ax.fill_between(x_dist_interp, tree_limit, safety_limit,    color="skyblue", alpha=0.3, label='Safety Margin')
            profile_ax.plot(x_dist_interp, terrain_interp, color='gray',  alpha=0.6, linewidth=1, label='Terrain')
            profile_ax.plot(x_dist_interp, fly_interp,     color='blue',  linewidth=2,            label='Flight Plan')

            # 绘制 Waypoint 蓝点 (横坐标现在是真实距离了！)
            fly_waypoints = [current_map.get_fly_height(*pt) for pt in path]
            profile_ax.scatter(cum_distances, fly_waypoints, color='blue', s=25, zorder=5, label='Waypoints')
            
            profile_ax.set_xlabel("Distance (m)")
            profile_ax.set_ylabel("Elevation (m)")
            profile_ax.legend(loc='upper right', fontsize='xx-small')
        except:
            pass # 如果没有对应的右图则跳过

    # --- 5. 绘制回程和其他标记 (保持原逻辑) ---
    if data_dict.get('path_retour'):
        px_ret = [dataset.index(pt[0], pt[1]) for pt in data_dict['path_retour']]
        ret_color = 'black'
        ax.plot([p[1] for p in px_ret], [p[0] for p in px_ret], color=ret_color, 
                 linestyle='--', linewidth=1, label='Retour (A*)', zorder=2)
        _add_arrows_to_path(ax, px_ret, ret_color)

    if data_dict.get('points_out'):
        pts = data_dict['points_out']
        if not is_dynamic:
            start_px = dataset.index(pts[0][0], pts[0][1])
            ax.scatter(start_px[1], start_px[0], color='#0000FF', s=60, edgecolors='white', label='Point A', zorder=6)
            if len(pts) > 1:
                others = [dataset.index(p[0], p[1]) for p in pts[1:]]
                ax.scatter([p[1] for p in others], [p[0] for p in others], color='red', s=25, edgecolors='white', label='Cibles', zorder=5)
        else:
            curr_px = dataset.index(pts[0][0], pts[0][1])
            ax.scatter(curr_px[1], curr_px[0], color='#00FF00', s=60, edgecolors='white', label='Drone Pos', zorder=6)
            # ... 其余 dynamic 逻辑保持不变 ...

    ax.set_title(title)
    ax.legend(loc='upper right', fontsize='xx-small')

def map_mutli_points_plot_compare(dataset, map_shape, data_before, data_after, current_map):
    """
    显示任务对比：左侧为地图视图，右侧为基于真实物理距离的高度剖面图。
    """
    # 创建 2x2 的布局：第一行是初始任务，第二行是动态任务
    # [0,0] 地图 [0,1] 剖面
    # [1,0] 地图 [1,1] 剖面
    fig, axs = plt.subplots(2, 2, figsize=(16, 12), gridspec_kw={'width_ratios': [1, 1.5]})
    
    titles = ["1. Trajet INITIAL", "2. Trajet DYNAMIQUE"]
    datasets = [data_before, data_after]
    is_dynamics = [False, True]

    for i in range(2):
        data_dict = datasets[i]
        # --- 1. 绘制左侧地图 ---
        # 调用你之前修改过的 _draw_mission_on_ax
        _draw_mission_on_ax(axs[i, 0], dataset, map_shape, data_dict, titles[i], is_dynamic=is_dynamics[i])

        # --- 2. 绘制右侧剖面图 (重点修改：使用物理距离) ---
        path = data_dict.get('path_aller', [])
        if path:
            # A. 计算每个 Waypoint 之间的物理距离 (累加)
            cum_distances = [0]
            curr_dist = 0
            for k in range(1, len(path)):
                # 计算欧几里得距离 (如果是经纬度，linalg.norm 结果是度，建议用 current_map 里的投影转换)
                p1 = np.array(path[k-1])
                p2 = np.array(path[k])
                dist = np.linalg.norm(p1 - p2) 
                curr_dist += dist
                cum_distances.append(curr_dist)
            cum_distances = np.array(cum_distances)

            # B. 高密度采样：为了让地形线(Terrain)看起来平滑
            x_interp = []      # 物理距离轴
            y_terrain = []     # 地形高度
            y_fly = []         # 飞行计划高度
            
            samples_per_segment = 10
            for k in range(len(path) - 1):
                p_start = np.array(path[k])
                p_end = np.array(path[k+1])
                dist_start = cum_distances[k]
                dist_end = cum_distances[k+1]

                # 在这一段距离内均匀取点
                for s in range(samples_per_segment):
                    ratio = s / samples_per_segment
                    interp_pt = p_start + ratio * (p_end - p_start)
                    interp_dist = dist_start + ratio * (dist_end - dist_start)
                    
                    x_interp.append(interp_dist)
                    y_terrain.append(current_map.get_elevation(*interp_pt))
                    y_fly.append(current_map.get_fly_height(*interp_pt))
            
            # 加上最后一个点
            x_interp.append(cum_distances[-1])
            y_terrain.append(current_map.get_elevation(*path[-1]))
            y_fly.append(current_map.get_fly_height(*path[-1]))

            # C. 开始绘图 (右侧子图 axs[i, 1])
            ax_prof = axs[i, 1]
            ax_prof.clear()
            
            # 绘制填充区域 (树木高度、安全冗余)
            y_terrain = np.array(y_terrain)
            y_fly = np.array(y_fly)
            tree_limit = y_terrain + current_map.max_tree_height
            
            ax_prof.fill_between(x_interp, y_terrain, tree_limit, color="green", alpha=0.2, label='Arbres')
            ax_prof.fill_between(x_interp, tree_limit, y_fly, color="skyblue", alpha=0.3, label='Marge de sécurité')
            
            # 绘制主线
            ax_prof.plot(x_interp, y_terrain, color='gray', alpha=0.8, label='Terrain')
            ax_prof.plot(x_interp, y_fly, color='blue', linewidth=2, label='Plan de Vol')
            
            # 绘制蓝色的 Waypoints (关键：使用累加后的真实距离)
            waypoint_heights = [current_map.get_fly_height(*pt) for pt in path]
            ax_prof.scatter(cum_distances, waypoint_heights, color='blue', s=30, zorder=5, edgecolors='white')

            ax_prof.set_title(f"Profil d'élévation - {titles[i]}")
            ax_prof.set_xlabel("Distance cumulée (unité carte)")
            ax_prof.set_ylabel("Altitude (m)")
            ax_prof.legend(loc='upper right', fontsize='x-small')
            ax_prof.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.show()

def map_mutli_points_plot(dataset, map_shape, points_in_shape=None, retour_path=None, points_out_shape=None, draw_path=True):
    fig, ax = plt.subplots(figsize=(10, 8))
    data = {'path_aller': points_in_shape, 'path_retour': retour_path, 'points_out': points_out_shape}
    _draw_mission_on_ax(ax, dataset, map_shape, data, "Mission Drone", is_dynamic=False)
    plt.show()

def mutli_points_plot_path(current_map, paths):
    fig, axs = plt.subplots(len(paths), 2, squeeze=False, figsize=(14, 4*len(paths)))
    dataset = current_map.dataset
    full_img = dataset.read(1)
    
    for i, path in enumerate(paths):
        # 左侧：地图
        pixels = [dataset.index(lon, lat) for lon, lat in current_map.map_shape]
        axs[i, 0].plot([p[1] for p in pixels], [p[0] for p in pixels], color='cyan', linewidth=2)
        
        pixels_in = [dataset.index(lon, lat) for lon, lat in path]
        axs[i, 0].plot([p[1] for p in pixels_in], [p[0] for p in pixels_in], color='#00FF00', linewidth=1.2, marker='.')
        axs[i, 0].imshow(full_img, cmap='gray')
        
        # 右侧：高度剖面
        # 这里移除原有的 n=10 循环采样，改为点对点直接提取
        fly_path = np.array([current_map.get_fly_height(*pt) for pt in path])
        terrain_h = np.array([current_map.get_elevation(*pt) for pt in path])
        
        x_axis = np.arange(len(path))
        
        axs[i, 1].plot(x_axis, terrain_h, label='Terrain', color='brown', alpha=0.4)
        axs[i, 1].plot(x_axis, fly_path, label='Flight Plan (Simplified)', color='blue', linewidth=2)
        
        # 填充安全区间
        axs[i, 1].fill_between(x_axis, terrain_h + current_map.max_tree_height + current_map.security_height, 
                               terrain_h, color="skyblue", alpha=0.3)
        
        axs[i, 1].set_title(f"Route {i+1} Elevation Control")
        axs[i, 1].legend(loc='upper right')
        
    plt.tight_layout()
    plt.show()

def dataset_plot(dataset):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    full_img = dataset.read()
    nir=dataset.read(dataset.count)

    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray -Band 1')

    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1 '.format(nir.shape))
    plt.show()

def map_plot(dataset, map_shape,points_in_shape=None, points_out_shape=None, draw_path = False):
    

    full_img = dataset.read(1)

    #plt.figure(figsize=(10, 8))

    pixels = [dataset.index(lon, lat) for lon, lat in map_shape]
    rows = [p[0] for p in pixels]
    cols = [p[1] for p in pixels]
    plt.plot(cols, rows, color='cyan', linewidth=2, label='Mission Boundary', zorder=2)

    if points_in_shape is not None and len(points_in_shape) > 0:
        pixels_in = [dataset.index(lon, lat) for lon, lat in points_in_shape]
        rows_in = [p[0] for p in pixels_in]
        cols_in = [p[1] for p in pixels_in]

    if points_out_shape is not None:
        pixels_out = [dataset.index(lon, lat) for lon, lat in points_out_shape]
        rows_out = [p[0] for p in pixels_out]
        cols_out = [p[1] for p in pixels_out]

    if draw_path:
        plt.plot(cols_in, rows_in, color='#00FF00', linewidth=1.5, 
                 marker='o', markersize=3, label='A* Flight Path', zorder=3)
        plt.scatter(cols_out, rows_out, color='red', s=50, edgecolors='white', 
                    label='Start/End Points', zorder=4)
        plt.title("UAV Path Planning - A* Algorithm Visualization")
    else :
        plt.scatter(cols_out, rows_out, color='red', alpha=0.4)
        plt.scatter(cols_in, rows_in, color='#00FF00', alpha=0.6)
        plt.title("Points in/out Map Shape")

    plt.imshow(full_img, cmap='gray') 
    plt.colorbar(label='Elevation (m)')
    
    plt.xlabel("Pixel Column")
    plt.ylabel("Pixel Row")
    plt.legend(loc='upper right')
    
    """if points_in_shape:
        margin = 50
        plt.xlim(min(cols_in) - margin, max(cols_in) + margin)
        plt.ylim(max(rows_in) + margin, min(rows_in) - margin) """

    plt.show()

def height_plot(heigth_array, security_height, max_tree_height, max_fly_height, fly_array_y = None, fly_array_x = None):
    plt.plot(heigth_array, label='terrain heigth', alpha=0.4)
    if fly_array_y is not None:
        if fly_array_x is not None:
            plt.plot(fly_array_x, fly_array_y, label='flight plan')
        else: 
            plt.plot(fly_array_y, label='flight plan')
    plt.plot(max_tree_height + heigth_array, color="darkgreen", label='terrain + tree', alpha=0.4)
    plt.fill_between(range(len(heigth_array)), max_tree_height + security_height + heigth_array, color="skyblue", alpha=0.4, label='terrain + tree + security')
    plt.axhline(y=max_fly_height, color='r', label='max_fly_height')
    plt.legend(loc='upper right')
    plt.show()

def mutliplot_path(current_map, paths, titles):
    """
    Affiche pour chaque trajet :
      - Colonne gauche  : vue carte (image + tracé)
      - Colonne droite  : profil d'altitude en fonction de la DISTANCE réelle (mètres)

    Interactivité :
      - En survolant le profil (droite), un marqueur croisé apparaît sur la carte (gauche)
        à la position correspondante du drone.
      - En survolant la carte (gauche), une ligne verticale suit la distance sur le profil.
    """
    matplotlib.use('TkAgg')   # assure un backend interactif ; à commenter si déjà défini

    n_rows = len(paths)
    fig, axs = plt.subplots(
        n_rows, 2, squeeze=False,
        figsize=(14, 5 * n_rows),
        gridspec_kw={"width_ratios": [1.6, 3]}
    )
    fig.patch.set_facecolor('#1a1a2e')

    dataset    = current_map.dataset
    full_img   = dataset.read(1)
    n_interp   = 30   # échantillons par segment pour le profil

    # Stocke les données par ligne pour les callbacks
    row_data = []

    for i, path in enumerate(paths):
        if not path:
            row_data.append(None)
            continue

        ax_map  = axs[i, 0]
        ax_prof = axs[i, 1]

        # ── Couleurs ──────────────────────────────────────────────────
        C_BOUND  = '#00e5ff'
        C_PATH   = '#76ff03'
        C_PROF   = '#448aff'
        C_TERRAIN= '#9e9e9e'

        # ── Gauche : carte ────────────────────────────────────────────
        ax_map.imshow(full_img, cmap='gray', alpha=0.85)
        boundary_px = [dataset.index(lon, lat) for lon, lat in current_map.map_shape]
        ax_map.plot(
            [p[1] for p in boundary_px] + [boundary_px[0][1]],
            [p[0] for p in boundary_px] + [boundary_px[0][0]],
            color=C_BOUND, linewidth=1.8, zorder=2
        )
        path_px = [dataset.index(pt[0], pt[1]) for pt in path]
        ax_map.plot(
            [p[1] for p in path_px], [p[0] for p in path_px],
            color=C_PATH, linewidth=1.6, zorder=3
        )
        # Waypoints
        ax_map.scatter(
            [p[1] for p in path_px], [p[0] for p in path_px],
            color=C_PATH, s=18, zorder=4
        )
        # Départ (vert) / Arrivée (rouge)
        ax_map.scatter(path_px[0][1],  path_px[0][0],  color='lime',  s=60, zorder=5, label='Départ')
        ax_map.scatter(path_px[-1][1], path_px[-1][0], color='red',   s=60, zorder=5, label='Arrivée')
        ax_map.set_title(f"Carte – trajet {i+1}", color='white', fontsize=9)
        ax_map.set_xticks([]); ax_map.set_yticks([])
        ax_map.legend(loc='upper left', fontsize=7, facecolor='#222', labelcolor='white',
                      bbox_to_anchor=(0.0, 1.0), borderaxespad=0)
        ax_map.set_facecolor('#111')

        # ── Zoom automatique sur le tracé (+ marge 20 %) ──────────────
        all_cols = [p[1] for p in path_px] + [p[1] for p in boundary_px]
        all_rows = [p[0] for p in path_px] + [p[0] for p in boundary_px]
        c_min, c_max = min(all_cols), max(all_cols)
        r_min, r_max = min(all_rows), max(all_rows)
        margin_c = max((c_max - c_min) * 0.20, 10)
        margin_r = max((r_max - r_min) * 0.20, 10)
        ax_map.set_xlim(c_min - margin_c, c_max + margin_c)
        ax_map.set_ylim(r_max + margin_r, r_min - margin_r)  # axe Y inversé pour imshow

        # ── Calcul distance cumulée (coordonnées projetées → mètres) ──
        # Les coordonnées stockées dans path sont déjà en CRS projeté (mètres)
        cum_dist = [0.0]
        for k in range(1, len(path)):
            p1 = np.array(path[k - 1])
            p2 = np.array(path[k])
            cum_dist.append(cum_dist[-1] + float(np.linalg.norm(p2 - p1)))
        cum_dist = np.array(cum_dist)

        # ── Interpolation dense du profil ─────────────────────────────
        x_dist, y_terrain, y_fly = [], [], []
        # Aussi stocker les coordonnées interpolées pour le repère carte
        interp_coords = []

        for k in range(len(path) - 1):
            p_s = np.array(path[k]);     p_e = np.array(path[k + 1])
            d_s = cum_dist[k];           d_e = cum_dist[k + 1]
            for j in range(n_interp):
                r = j / n_interp
                pt  = p_s + r * (p_e - p_s)
                dist = d_s + r * (d_e - d_s)
                x_dist.append(dist)
                y_terrain.append(current_map.get_elevation(pt[0], pt[1]))

                h_lin  = current_map.get_fly_height(*p_s) * (1 - r) + current_map.get_fly_height(*p_e) * r
                h_safe = current_map.get_fly_height(pt[0], pt[1])
                y_fly.append(max(h_lin, h_safe))
                interp_coords.append((pt[0], pt[1]))

        # Dernier point
        x_dist.append(float(cum_dist[-1]))
        y_terrain.append(current_map.get_elevation(*path[-1]))
        h_last = current_map.get_fly_height(*path[-1])
        y_fly.append(h_last)
        interp_coords.append((path[-1][0], path[-1][1]))

        x_dist    = np.array(x_dist)
        y_terrain = np.array(y_terrain)
        y_fly     = np.array(y_fly)

        tree_limit   = y_terrain + current_map.max_tree_height
        safety_limit = tree_limit + current_map.security_height

        # ── Droite : profil ────────────────────────────────────────────
        ax_prof.set_facecolor('#0d1117')
        ax_prof.fill_between(x_dist, y_terrain, tree_limit,
                             color='#2e7d32', alpha=0.35, label='Arbres')
        ax_prof.fill_between(x_dist, tree_limit, safety_limit,
                             color='#0288d1', alpha=0.25, label='Marge sécurité')
        ax_prof.plot(x_dist, y_terrain, color=C_TERRAIN, alpha=0.7, linewidth=1, label='Terrain')
        ax_prof.plot(x_dist, y_fly,     color=C_PROF,    linewidth=2,             label='Plan de vol')

        # Waypoints sur le profil (distance réelle)
        wp_heights = [current_map.get_fly_height(*pt) for pt in path]
        ax_prof.scatter(cum_dist, wp_heights, color=C_PROF, s=30, zorder=5,
                        edgecolors='white', linewidths=0.5)

        ax_prof.axhline(y=current_map.max_fly_height, color='red',
                        linestyle='--', alpha=0.7, linewidth=1, label='Alt. max')

        total_m = cum_dist[-1]
        ax_prof.set_title(
            f"Profil {i+1} | {len(path)} waypoints | {total_m:.0f} m | {titles[i]}",
            color='white', fontsize=9
        )
        ax_prof.set_xlabel("Distance (m)", color='#ccc')
        ax_prof.set_ylabel("Altitude (m)", color='#ccc')
        ax_prof.tick_params(colors='#aaa')
        ax_prof.legend(loc='lower left', fontsize=7, facecolor='#222', labelcolor='white',
                      framealpha=0.85, edgecolor='#555')
        ax_prof.grid(True, linestyle='--', alpha=0.2, color='#555')
        for spine in ax_prof.spines.values():
            spine.set_edgecolor('#444')

        # ── Curseurs interactifs ───────────────────────────────────────
        # Marqueur mobile sur la carte
        map_marker, = ax_map.plot([], [], 'o', color='yellow',
                                  markersize=10, zorder=10,
                                  markeredgecolor='black', markeredgewidth=1)
        map_crossH  = ax_map.axhline(y=-999, color='yellow', alpha=0.4, linewidth=0.8)
        map_crossV  = ax_map.axvline(x=-999, color='yellow', alpha=0.4, linewidth=0.8)

        # Ligne verticale mobile sur le profil
        prof_vline  = ax_prof.axvline(x=-999, color='yellow', alpha=0.5, linewidth=1)
        prof_dot,   = ax_prof.plot([], [], 'o', color='yellow', markersize=7, zorder=10,
                                   markeredgecolor='black', markeredgewidth=0.8)
        # Annotation altitude
        prof_annot  = ax_prof.annotate('', xy=(0, 0), xytext=(8, 8),
                                       textcoords='offset points',
                                       fontsize=7, color='yellow',
                                       bbox=dict(boxstyle='round,pad=0.3',
                                                 fc='#1a1a2e', ec='yellow', alpha=0.85))

        row_data.append({
            'path': path,
            'path_px': path_px,
            'cum_dist': cum_dist,
            'x_dist': x_dist,
            'y_fly': y_fly,
            'interp_coords': interp_coords,
            'ax_map': ax_map,
            'ax_prof': ax_prof,
            'map_marker': map_marker,
            'map_crossH': map_crossH,
            'map_crossV': map_crossV,
            'prof_vline': prof_vline,
            'prof_dot': prof_dot,
            'prof_annot': prof_annot,
        })

    # ── Callback souris ───────────────────────────────────────────────
    def _update_from_dist(rd, dist_m):
        """Met à jour les deux axes à partir d'une distance (m)."""
        idx = int(np.searchsorted(rd['x_dist'], dist_m, side='left'))
        idx = max(0, min(idx, len(rd['x_dist']) - 1))

        coord = rd['interp_coords'][idx]
        px    = dataset.index(coord[0], coord[1])
        alt   = rd['y_fly'][idx]
        d     = rd['x_dist'][idx]

        # Carte
        rd['map_marker'].set_data([px[1]], [px[0]])
        rd['map_crossH'].set_ydata([px[0]])
        rd['map_crossV'].set_xdata([px[1]])
        # Profil
        rd['prof_vline'].set_xdata([d])
        rd['prof_dot'].set_data([d], [alt])
        rd['prof_annot'].set_text(f"{alt:.1f} m\n{d:.0f} m")
        rd['prof_annot'].xy = (d, alt)

    def _update_from_map_px(rd, col, row):
        """Met à jour à partir d'un clic/survol sur la carte (pixel)."""
        # Trouve le waypoint le plus proche en pixel
        dists_px = [np.hypot(col - p[1], row - p[0]) for p in rd['path_px']]
        wi = int(np.argmin(dists_px))
        dist_m = float(rd['cum_dist'][wi])
        _update_from_dist(rd, dist_m)

    def on_mouse_move(event):
        for rd in row_data:
            if rd is None:
                continue
            if event.inaxes == rd['ax_prof']:
                if event.xdata is not None:
                    _update_from_dist(rd, event.xdata)
                    fig.canvas.draw_idle()
            elif event.inaxes == rd['ax_map']:
                if event.xdata is not None and event.ydata is not None:
                    _update_from_map_px(rd, event.xdata, event.ydata)
                    fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

    plt.tight_layout(pad=1.5)
    plt.subplots_adjust(hspace=0.45)
    plt.show()

def get_cumulative_distances(path):
    distances = [0]
    total_dist = 0
    for i in range(1, len(path)):
        # 计算相邻两点间的欧几里得距离
        p1 = np.array(path[i-1])
        p2 = np.array(path[i])
        dist = np.linalg.norm(p1 - p2)
        total_dist += dist
        distances.append(total_dist)
    return np.array(distances)