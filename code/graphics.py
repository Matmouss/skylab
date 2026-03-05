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