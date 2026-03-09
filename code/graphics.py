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
    nir=dataset.read(dataset.count)

    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray -Band 1')

    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1 '.format(nir.shape))
    plt.show()

def map_plot(dataset, map_shape, points_in_shape=None, retour_path=None, points_out_shape=None, draw_path=True):
    full_img = dataset.read(1)

    # 1. Limites de mission (Cyan)
    pixels = [dataset.index(lon, lat) for lon, lat in map_shape]
    plt.plot([p[1] for p in pixels], [p[0] for p in pixels], color='cyan', linewidth=2, label='Limites', zorder=2)

    # 2. Trajet ALLER (Cyan vif)
    if points_in_shape:
        px_in = [dataset.index(pt[0], pt[1]) for pt in points_in_shape]
        plt.plot([p[1] for p in px_in], [p[0] for p in px_in], color='#00FFFF', 
                 linewidth=1.5, marker='o', markersize=1, label='Trajet Aller', zorder=3)

    # 3. Trajet RETOUR (Gris Pointillé) - 灰色虚线
    if retour_path:
        px_ret = [dataset.index(pt[0], pt[1]) for pt in retour_path]
        plt.plot([p[1] for p in px_ret], [p[0] for p in px_ret], color='gray', 
                 linestyle='--', linewidth=1.5, label='Trajet Retour (A*)', zorder=2)

    # 4. Points de repère (Taille réduite)
    if points_out_shape:
        # Départ A (Vert) 
        start_pt = points_out_shape[0]
        st_px = dataset.index(start_pt[0], start_pt[1])
        plt.scatter(st_px[1], st_px[0], color='#00FF00', s=40, edgecolors='white', label='Départ (A)', zorder=5)
        
        # Autres points (Rouge) 
        if len(points_out_shape) > 1:
            others = [dataset.index(p[0], p[1]) for p in points_out_shape[1:]]
            plt.scatter([p[1] for p in others], [p[0] for p in others], color='red', s=20, edgecolors='white', label='Cibles', zorder=4)

    plt.imshow(full_img, cmap='gray')
    plt.legend(loc='upper right', fontsize='small')
    plt.title("Mission Drone : Multi-points avec retour A*")
    plt.show()

def mutliplot_path(current_map, paths):
    fig, axs = plt.subplots(len(paths), 2, squeeze=False)
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

        # profil
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
        axs[i, 1].legend(loc='upper right')

    plt.tight_layout()
    plt.show()