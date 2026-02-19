import rasterio
from pyproj import Transformer
import os

def get_elevation(tif_path, coord_a, coord_b):
    """
    Fonction universelle pour extraire l'altitude.
    :param tif_path: Chemin vers le fichier .tif (MNT)
    :param coord_a: Latitude (GPS) ou X (projection en mètres)
    :param coord_b: Longitude (GPS) ou Y (projection en mètres)
    """
    if not os.path.exists(tif_path):
        return f"❌ Erreur : Le fichier {tif_path} est introuvable."

    with rasterio.open(tif_path) as src:
        # 1. Identification automatique du type de coordonnées
        # Si les valeurs sont petites (entre -180 et 180), on considère que c'est du GPS (WGS84)
        if abs(coord_a) < 1000 and abs(coord_b) < 1000:
            lat, lon = coord_a, coord_b
            # Création du transformateur : GPS (EPSG:4326) -> Projection du TIF (ex: EPSG:3035)
            transformer = Transformer.from_crs("EPSG:4326", src.crs, always_xy=True)
            target_x, target_y = transformer.transform(lon, lat)
            print(f"📡 Coordonnées GPS détectées. Conversion en coordonnées projetées : X={target_x:.2f}, Y={target_y:.2f}")
        else:
            # Sinon, on considère que ce sont déjà des coordonnées en mètres
            target_x, target_y = coord_a, coord_b
            print(f"📏 Coordonnées en mètres détectées : X={target_x:.2f}, Y={target_y:.2f}")

        # 2. Vérification de l'emprise (Bounding Box)
        b = src.bounds
        if not (b.left <= target_x <= b.right and b.bottom <= target_y <= b.top):
            return (f"❌ Hors limites ! Les coordonnées sont en dehors de l'emprise du fichier TIF.\n"
                    f"   Emprise du fichier : X[{b.left:.1f}, {b.right:.1f}], Y[{b.bottom:.1f}, {b.top:.1f}]")

        # 3. Extraction de l'altitude
        # 'index' convertit les coordonnées projetées en indices de matrice (ligne, colonne)
        row, col = src.index(target_x, target_y)
        raster_data = src.read(1)
        elevation = raster_data[row, col]

        return f"✅ Succès ! Altitude : {elevation:.2f} mètres"

def get_tif_center_gps(tif_path):
    """Outil auxiliaire : calcule les coordonnées GPS du centre du fichier TIF."""
    with rasterio.open(tif_path) as src:
        cx = (src.bounds.left + src.bounds.right) / 2
        cy = (src.bounds.bottom + src.bounds.top) / 2
        # Transformation inverse : Projection locale -> GPS (EPSG:4326)
        transformer = Transformer.from_crs(src.crs, "EPSG:4326", always_xy=True)
        lon, lat = transformer.transform(cx, cy)
        return lat, lon

# ==========================================
# Point d'entrée principal du programme
# ==========================================
if __name__ == "__main__":
    # Assurez-vous que le chemin est correct
    PATH = "data/output_be2.tif" 
    
    print("--- Lecture des métadonnées du fichier ---")
    try:
        # Calcul du centre pour éviter les erreurs de coordonnées "hors limites"
        #c_lat, c_lon = get_tif_center_gps(PATH)
        c_lat, c_lon = 47.275,-1.525
        print(f"💡 Centre géographique du fichier (GPS) pour tester : {c_lat:.6f}, {c_lon:.6f}")
        print(f"🔗 Voir sur Google Maps : https://www.google.com/maps?q={c_lat},{c_lon}")
        
        print("\n--- Test de requête d'altitude ---")
        
        # Test 1 : Utilisation du centre géographique (doit fonctionner à 100%)
        print("Test 1 (Point central) :")
        print(get_elevation(PATH, c_lat, c_lon))

    except Exception as e:
        print(f"❌ Échec de l'exécution : {e}")