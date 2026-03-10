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

        if data_dict.get('path_retour'):
            px_ret = [dataset.index(pt[0], pt[1]) for pt in data_dict['path_retour']]
            ret_color = 'black'
            ax.plot([p[1] for p in px_ret], [p[0] for p in px_ret], color=ret_color, 
                     linestyle='--', linewidth=1, label='Retour (A*)', zorder=2)
            _add_arrows_to_path(ax, px_ret, ret_color)

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

def map_mutli_points_plot_compare(dataset, map_shape, data_before, data_after):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
    _draw_mission_on_ax(ax1, dataset, map_shape, data_before, "1. Trajet INITIAL", is_dynamic=False)
    _draw_mission_on_ax(ax2, dataset, map_shape, data_after, "2. Trajet DYNAMIQUE", is_dynamic=True)
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
    len(paths),
    2,
    squeeze=False,
    figsize=(12, 4*len(paths)),
    gridspec_kw={"width_ratios": [1.8, 3]}  # gauche plus large
    )
    dataset = current_map.dataset
    full_img = dataset.read(1)

    for i, path in enumerate(paths):
        # carte
        pixels = [dataset.index(lon, lat) for lon, lat in current_map.map_shape]
        rows = [p[0] for p in pixels]
        cols = [p[1] for p in pixels]
        axs[i, 0].plot(cols, rows, color='cyan', linewidth=2, label='Mission Boundary', zorder=2)

        pixels_in = [dataset.index(lon, lat) for lon, lat in path]
        rows_in = [p[0] for p in pixels_in]
        cols_in = [p[1] for p in pixels_in]
        axs[i, 0].plot(cols_in, rows_in, color='#00FF00', linewidth=1.5,
                       marker='o', markersize=3, label='Flight Path', zorder=3)

        axs[i, 0].set_title(f"A* Visualization {i+1}")
        im = axs[i, 0].imshow(full_img, cmap='gray')
        #fig.colorbar(im, ax=axs[i, 0], label='Elevation (m)')
        axs[i, 0].legend(loc='upper right')
        axs[i,0].set_xticks([])
        axs[i,0].set_yticks([])

        # profil
        fly_path = np.array([current_map.get_fly_height(*pt) for pt in path])

        n = 1
        heigth_array = []
        for k in range(len(path) - 1):
            x = np.linspace(path[k][0], path[k + 1][0], n, endpoint=False)
            y = np.linspace(path[k][1], path[k + 1][1], n, endpoint=False)
            for j in range(n):
                heigth_array.append(current_map.get_elevation(x[j], y[j]))

        heigth_array.append(current_map.get_elevation(path[-1][0], path[-1][1]))
        heigth_array = np.array(heigth_array)

        fly_array_x = np.arange(len(path)) * n

        axs[i, 1].plot(heigth_array, label='terrain height', alpha=0.4)
        axs[i, 1].plot(fly_array_x, fly_path, label='flight plan')
        axs[i, 1].plot(current_map.max_tree_height + heigth_array, color="darkgreen",
                       label='terrain + tree', alpha=0.4)
        axs[i, 1].fill_between(
            range(len(heigth_array)),
            current_map.max_tree_height + current_map.security_height + heigth_array,
            color="skyblue", alpha=0.4, label='terrain + tree + security'
        )
        axs[i, 1].axhline(y=current_map.max_fly_height, color='r', label='max_fly_height')
        axs[i, 1].set_title(f"Profil {i+1} : {len(path)} points | {titles[i]}")
        axs[i, 1].legend(loc='upper right')
        axs[i,1].set_xticks([])
    plt.tight_layout()
    plt.show()
  