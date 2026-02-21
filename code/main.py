import topography, graphics
import os

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)

    tif_path = r"..\data\output_be.tif" 

    current_map = topography.Map(tif_path)

    print(current_map.get_elevation(3451535.0,2752425.0))
