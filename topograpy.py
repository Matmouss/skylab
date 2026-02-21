import rasterio 
import numpy as np
from pyproj import Transformer
import os

#visuel
import shapely
from shapely import geometry
from shapely.geometry import shape, Point, LineString, Polygon
from rasterio.plot import show
import matplotlib 
import matplotlib.pyplot as plt


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

def get_elevation(src, coord_a, coord_b):
    """
    Si les valeurs sont petites (entre -180 et 180), on considère que c'est du GPS (WGS84)
        --> Création du transformateur : GPS (EPSG:4326) -> Projection du TIF (ex: EPSG:3035)
    Sinon, on considère que ce sont déjà des coordonnées en mètres
    """
    if abs(coord_a) < 1000 and abs(coord_b) < 1000:
        lat, lon = coord_a, coord_b
        transformer = Transformer.from_crs("EPSG:4326", src.crs, always_xy=True)
        target_x, target_y = transformer.transform(lon, lat)
    else:
        target_x, target_y = coord_a, coord_b

    # Vérification de l'emprise (Bounding Box)
    b = src.bounds
    if not (b.left <= target_x <= b.right and b.bottom <= target_y <= b.top):
        raise Exception(f"Hors limites ! Les coordonnées sont en dehors de l'emprise du fichier TIF.\n"
                f"   Emprise du fichier : X[{b.left:.1f}, {b.right:.1f}], Y[{b.bottom:.1f}, {b.top:.1f}]")

    # Extraction de l'altitude
    # 'index' convertit les coordonnées projetées en indices de matrice (ligne, colonne)
    row, col = src.index(target_x, target_y)
    raster_data = src.read(1)
    elevation = raster_data[row, col]

    return elevation


if __name__ == "__main__":
    tif_path = r"data\output_be.tif" 

    if not os.path.exists(tif_path):
        raise Exception(f"Erreur : Le fichier {tif_path} est introuvable.")

    dataset =  rasterio.open(tif_path)

    #dataset_info(dataset)
    #dataset_plot(dataset)

    print(get_elevation(dataset, 3451535.0,2752425.0))

    dataset.close()

