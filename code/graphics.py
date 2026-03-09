import shapely
from shapely import geometry
from shapely.geometry import shape, Point, LineString, Polygon
from rasterio.plot import show
import matplotlib 
import matplotlib.pyplot as plt
import numpy as np

def dataset_plot(dataset):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    full_img = dataset.read()
    nir = dataset.read(dataset.count)
    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray - Band 1')
    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1')
    plt.show()

def _add_arrows_to_path(ax, pixel_coords, color):
    """
    在路径的中间位置添加一个精简的指示方向的箭头。
    """
    if len(pixel_coords) < 5:  # 路径太短则不画
        return

    coords = np.array(pixel_coords)
    # 取路径中间的一个小线段来确定方向
    mid = len(coords) // 2
    r1, c1 = coords[mid]
    r2, c2 = coords[mid + 1]
    
    # mutation_scale 调小到 8，显得精致不突兀
    ax.annotate('', xy=(c2, r2), xytext=(c1, r1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1, mutation_scale=8),
                zorder=4)

def _draw_mission_on_ax(ax, dataset, map_shape, data_dict, title, is_dynamic=False):
    full_img = dataset.read(1)
    ax.imshow(full_img, cmap='gray')

    # 1. 绘制任务边界
    pixels = [dataset.index(lon, lat) for lon, lat in map_shape]
    ax.plot([p[1] for p in pixels], [p[0] for p in pixels], color='cyan', linewidth=2, label='Limites', zorder=2)

    if data_dict:
        # 2. 绘制去程路径
        if data_dict.get('path_aller'):
            px_in = [dataset.index(pt[0], pt[1]) for pt in data_dict['path_aller']]
            line_color = '#00FFFF'
            ax.plot([p[1] for p in px_in], [p[0] for p in px_in], color=line_color, 
                     linewidth=1.2, label='Trajet Aller', zorder=3)
            # 仅在中间画一个箭头
            _add_arrows_to_path(ax, px_in, line_color)

        # 3. 绘制回程路径
        if data_dict.get('path_retour'):
            px_ret = [dataset.index(pt[0], pt[1]) for pt in data_dict['path_retour']]
            ret_color = 'black'
            ax.plot([p[1] for p in px_ret], [p[0] for p in px_ret], color=ret_color, 
                     linestyle='--', linewidth=1, label='Retour (A*)', zorder=2)
            _add_arrows_to_path(ax, px_ret, ret_color)

        # 4. 点标注
        if data_dict.get('points_out'):
            pts = data_dict['points_out']
            if not is_dynamic:
                # 初始状态：蓝色起点
                start_px = dataset.index(pts[0][0], pts[0][1])
                ax.scatter(start_px[1], start_px[0], color='#0000FF', s=60, edgecolors='white', label='Point A', zorder=6)
                if len(pts) > 1:
                    others = [dataset.index(p[0], p[1]) for p in pts[1:]]
                    ax.scatter([p[1] for p in others], [p[0] for p in others], color='red', s=25, edgecolors='white', label='Cibles', zorder=5)
            else:
                # 动态状态：绿色当前位置，蓝色原始起点
                curr_px = dataset.index(pts[0][0], pts[0][1])
                ax.scatter(curr_px[1], curr_px[0], color='#00FF00', s=60, edgecolors='white', label='Drone Pos', zorder=6)
                if len(pts) > 1:
                    origin_px = dataset.index(pts[1][0], pts[1][1])
                    ax.scatter(origin_px[1], origin_px[0], color='#0000FF', s=40, edgecolors='white', label='Point A', zorder=5)
                if len(pts) > 2:
                    others = [dataset.index(p[0], p[1]) for p in pts[2:]]
                    ax.scatter([p[1] for p in others], [p[0] for p in others], color='red', s=25, edgecolors='white', label='Cibles', zorder=5)

    ax.set_title(title)
    ax.legend(loc='upper right', fontsize='xx-small')

def map_plot_compare(dataset, map_shape, data_before, data_after):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
    _draw_mission_on_ax(ax1, dataset, map_shape, data_before, "1. Trajet INITIAL", is_dynamic=False)
    _draw_mission_on_ax(ax2, dataset, map_shape, data_after, "2. Trajet DYNAMIQUE", is_dynamic=True)
    plt.tight_layout()
    plt.show()

def map_plot(dataset, map_shape, points_in_shape=None, retour_path=None, points_out_shape=None, draw_path=True):
    fig, ax = plt.subplots(figsize=(10, 8))
    data = {'path_aller': points_in_shape, 'path_retour': retour_path, 'points_out': points_out_shape}
    _draw_mission_on_ax(ax, dataset, map_shape, data, "Mission Drone", is_dynamic=False)
    plt.show()

def mutliplot_path(current_map, paths):
    fig, axs = plt.subplots(len(paths), 2, squeeze=False, figsize=(14, 4*len(paths)))
    dataset = current_map.dataset
    full_img = dataset.read(1)
    for i, path in enumerate(paths):
        pixels = [dataset.index(lon, lat) for lon, lat in current_map.map_shape]
        rows, cols = [p[0] for p in pixels], [p[1] for p in pixels]
        axs[i, 0].plot(cols, rows, color='cyan', linewidth=2, zorder=2)
        pixels_in = [dataset.index(lon, lat) for lon, lat in path]
        rows_in, cols_in = [p[0] for p in pixels_in], [p[1] for p in pixels_in]
        axs[i, 0].plot(cols_in, rows_in, color='#00FF00', linewidth=1.2, zorder=3)
        axs[i, 0].imshow(full_img, cmap='gray')
        
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
        axs[i, 1].plot(heigth_array, label='Terrain', alpha=0.4)
        axs[i, 1].plot(fly_array_x, fly_path, label='Flight Plan')
        axs[i, 1].legend(loc='upper right')
    plt.tight_layout()
    plt.show()