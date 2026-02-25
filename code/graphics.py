import shapely
from shapely import geometry
from shapely.geometry import shape, Point, LineString, Polygon
from rasterio.plot import show
import matplotlib 
import matplotlib.pyplot as plt


def dataset_plot(dataset):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    full_img = dataset.read()
    nir=dataset.read(dataset.count)

    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray -Band 1')

    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1 '.format(nir.shape))
    plt.show()

def map_plot(dataset, map_shape, points_in_shape=None, points_out_shape=None):
    full_img = dataset.read(1)

    # Conversion lon/lat → pixel
    pixels = [dataset.index(lon, lat) for lon, lat in map_shape]
    rows = [p[0] for p in pixels]
    cols = [p[1] for p in pixels]

    #convertir les points en pixels
    if points_in_shape is not None:
        pixels_in = [dataset.index(lon, lat) for lon, lat in points_in_shape]
        rows_in = [p[0] for p in pixels_in]
        cols_in = [p[1] for p in pixels_in]

    if points_out_shape is not None:
        pixels_out = [dataset.index(lon, lat) for lon, lat in points_out_shape]
        rows_out = [p[0] for p in pixels_out]
        cols_out = [p[1] for p in pixels_out]

    plt.scatter(cols_out, rows_out, color='red',alpha=0.4)
    plt.scatter(cols_in, rows_in, color='green', alpha=0.7)

    plt.imshow(full_img, cmap='gray')
    plt.plot(cols, rows, color='blue')
    plt.show()

def height_plot(heigth_array, security_height, max_tree_height, max_fly_height, fly_array = None):
    plt.plot(heigth_array, label='terrain heigth', alpha=0.4)
    if fly_array is not None:
        plt.plot(fly_array, label='flight plan')
    plt.plot(max_tree_height + heigth_array, color="darkgreen", label='terrain heigth + max_tree_height', alpha=0.4)
    plt.fill_between(range(len(heigth_array)), max_tree_height + security_height + heigth_array, color="skyblue", alpha=0.4, label='terrain heigth + max_tree_height + security_height')
    plt.axhline(y=max_fly_height, color='r', label='max_fly_height')
    plt.legend()
    plt.show()