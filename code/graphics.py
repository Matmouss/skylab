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
    fig, axs = plt.subplots(
        len(paths), 2, squeeze=False,
        figsize=(12, 4*len(paths)),
        gridspec_kw={"width_ratios": [1.8, 3]}
    )
    dataset = current_map.dataset
    full_img = dataset.read(1)
    n_interp = 20

    for i, path in enumerate(paths):
        # --- 左图 ---
        pixels = [dataset.index(lon, lat) for lon, lat in current_map.map_shape]
        axs[i, 0].plot([p[1] for p in pixels], [p[0] for p in pixels], color='cyan', linewidth=2)
        pixels_in = [dataset.index(lon, lat) for lon, lat in path]
        axs[i, 0].plot([p[1] for p in pixels_in], [p[0] for p in pixels_in],
                       color='#00FF00', linewidth=1.5, marker='o', markersize=3)
        axs[i, 0].imshow(full_img, cmap='gray')
        axs[i, 0].set_title(f"Map View {i+1}")
        axs[i, 0].set_xticks([])
        axs[i, 0].set_yticks([])

        # --- 右图：地形与飞行高度都做插值 ---
        terrain_interp = []
        fly_interp = []
        x_interp = []
        total_idx = 0

        for k in range(len(path) - 1):
            xs = np.linspace(path[k][0], path[k+1][0], n_interp, endpoint=False)
            ys = np.linspace(path[k][1], path[k+1][1], n_interp, endpoint=False)

            h_start = current_map.get_fly_height(*path[k])
            h_end   = current_map.get_fly_height(*path[k+1])

            for j in range(n_interp):
                terrain_interp.append(current_map.get_elevation(xs[j], ys[j]))

                t = j / n_interp
                linear_h  = h_start * (1 - t) + h_end * t
                min_safe_h = current_map.get_fly_height(xs[j], ys[j])
                fly_interp.append(max(linear_h, min_safe_h))

                x_interp.append(total_idx + j / n_interp)
            total_idx += 1

        # 终点
        terrain_interp.append(current_map.get_elevation(*path[-1]))
        fly_interp.append(current_map.get_fly_height(*path[-1]))
        x_interp.append(total_idx)

        terrain_interp = np.array(terrain_interp)
        fly_interp     = np.array(fly_interp)
        x_interp       = np.array(x_interp)

        tree_limit   = terrain_interp + current_map.max_tree_height
        safety_limit = tree_limit + current_map.security_height

        axs[i, 1].fill_between(x_interp, terrain_interp, tree_limit,  color="green",   alpha=0.2, label='Trees')
        axs[i, 1].fill_between(x_interp, tree_limit, safety_limit,    color="skyblue", alpha=0.3, label='Safety Margin')
        axs[i, 1].plot(x_interp, terrain_interp, color='gray',  alpha=0.6, linewidth=1, label='Terrain')
        axs[i, 1].plot(x_interp, fly_interp,     color='blue',  linewidth=2,            label='Flight Plan')

        # waypoint 标记点
        fly_waypoints = [current_map.get_fly_height(*pt) for pt in path]
        axs[i, 1].scatter(np.arange(len(path)), fly_waypoints, color='blue', s=20, zorder=5)

        axs[i, 1].axhline(y=current_map.max_fly_height, color='red', linestyle='--', alpha=0.7, label='Max Altitude')
        axs[i, 1].set_title(f"Profile {i+1} : {len(path)} Waypoints | {titles[i]}")
        axs[i, 1].legend(loc='upper right', fontsize='xx-small')
        axs[i, 1].set_xlabel("Waypoint Index")
        axs[i, 1].set_ylabel("Elevation (m)")

    plt.tight_layout()
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