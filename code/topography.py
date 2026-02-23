import rasterio 
import numpy as np
from pyproj import Transformer
import os

"""
open topography: https://portal.opentopography.org/raster?opentopoID=OTSDEM.092022.3035.1


Excration des données topographiques du fichier data/output_be.tif

https://rasterio.readthedocs.io/en/stable/
https://github.com/patrickcgray/open-geo-tutorial 

"""

class Map:
    def __init__(self, tif_path, max_tree_height = 20,security_height=5, max_fly_height=60):
        if not os.path.exists(tif_path):
            raise Exception(f"Erreur : Le fichier {tif_path} est introuvable.")
        self.tif_path = tif_path
        self.dataset = rasterio.open(tif_path)
        self.max_tree_height = max_tree_height
        self.security_height = security_height
        self.max_fly_height = max_fly_height
        self.raster_data = self.dataset.read(1)

    def __init__(self, config):
        if "tif_path" not in config or "security_height" not in config or "max_tree_height" not in config: raise Exception(f"Erreur : clée manquante dans le fichier de configuration.")
        self.tif_path = config["tif_path"]
        if not os.path.exists(self.tif_path): raise Exception(f"Erreur : Le fichier {self.tif_path} est introuvable.")
        self.dataset = rasterio.open(self.tif_path)
        self.max_tree_height = config["max_tree_height"]
        self.security_height = config["security_height"]
        self.max_fly_height = config["max_fly_height"]
        self.raster_data = self.dataset.read(1)

    def __del__(self):
        try:
            self.dataset.close()
        except:
            pass

    def get_elevation(self, coord_a, coord_b):
        """
        Si les valeurs sont petites (entre -180 et 180), on considère que c'est du GPS (WGS84)
            --> Création du transformateur : GPS (EPSG:4326) -> Projection du TIF (ex: EPSG:3035)
        Sinon, on considère que ce sont déjà des coordonnées en mètres
        """
        if abs(coord_a) < 1000 and abs(coord_b) < 1000:
            lat, lon = coord_a, coord_b
            transformer = Transformer.from_crs("EPSG:4326", self.dataset.crs, always_xy=True)
            target_x, target_y = transformer.transform(lon, lat)
        else:
            target_x, target_y = coord_a, coord_b

        # Vérification de l'emprise (Bounding Box)
        b = self.dataset.bounds
        if not (b.left <= target_x <= b.right and b.bottom <= target_y <= b.top):
            raise Exception(f"Hors limites ! Les coordonnées sont en dehors de l'emprise du fichier TIF.\n"
                    f"   Emprise du fichier : X[{b.left:.1f}, {b.right:.1f}], Y[{b.bottom:.1f}, {b.top:.1f}]")

        # Extraction de l'altitude
        # 'index' convertit les coordonnées projetées en indices de matrice (ligne, colonne)
        row, col = self.dataset.index(target_x, target_y)
        elevation = self.raster_data[row, col]

        return elevation
    
    # str a modifier les infos ne sont pas pertinentes pour le moment
    def __str__(self):
        ret = ""
        img_name = self.dataset.name
        ret += 'Image filename: {n}\n'.format(n=img_name)

        ret += 'Max tree height: {h} m\n'.format(h=self.max_tree_height)
        ret += 'Security height: {h} m\n'.format(h=self.security_height)

        ret += 'Projection: {p}\n'.format(p=self.raster_data)

        return ret

