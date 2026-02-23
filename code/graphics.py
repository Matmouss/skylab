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

def height_plot(heigth_array, security_height, max_tree_height, max_fly_height, fly_array = None):
    plt.plot(heigth_array, label='terrain heigth')
    if fly_array is not None:
        plt.plot(fly_array, label='fly')
    plt.plot(max_tree_height + heigth_array, 'g', label='terrain heigth + max_tree_height')
    plt.plot(max_tree_height + security_height + heigth_array, 'b', label='terrain heigth + max_tree_height + security_height')
    plt.axhline(y=max_fly_height, color='r', label='max_fly_height')
    plt.legend()
    plt.show()