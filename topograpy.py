import rasterio 
import matplotlib 
import matplotlib.pyplot as plt
import numpy as np
from rasterio.plot import show


import shapely
from shapely import geometry
from shapely.geometry import shape, Point, LineString, Polygon



"""
open topography: https://portal.opentopography.org/raster?opentopoID=OTSDEM.092022.3035.1


Excration des données topographiques du fichier data/output_be.tif

https://rasterio.readthedocs.io/en/stable/
https://github.com/patrickcgray/open-geo-tutorial 

"""

asc_path = "data/output_be.asc"
prj_path = "data/output_be.prj"
tif_path = "data/output_be.tif"

def dataset_info(dataset):
    img_name = dataset.name
    print('Image filename: {n}\n'.format(n=img_name))

    num_bands = dataset.count
    print('Number of bands in image: {n}\n'.format(n=num_bands))

    rows, cols = dataset.shape
    print('Image size is: {r} rows x {c} columns\n'.format(r=rows, c=cols))

    desc = dataset.descriptions
    metadata = dataset.meta

    print('Raster description: {desc}\n'.format(desc=desc))

    driver = dataset.driver
    print('Raster driver: {d}\n'.format(d=driver))

    proj = dataset.crs
    print('Image projection:')
    print(proj, '\n')

    gt = dataset.transform

    print('Image geo-transform:\n{gt}\n'.format(gt=gt))

    print('All raster metadata:')
    print(metadata)
    print('\n')

def dataset_plot(dataset):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    full_img = dataset.read()
    nir=dataset.read(dataset.count)

    ax1.imshow(nir, cmap='gray')
    ax1.set_title('gray -Band 1')

    ax2.imshow(full_img[0, :, :])
    ax2.set_title('color - Band 1 '.format(nir.shape))
    plt.show()



if __name__ == "__main__":
    dataset =  rasterio.open(tif_path)

    dataset_info(dataset)

    #dataset_plot(dataset)

    full_img = dataset.read()
    print(full_img)
    dataset.close()

