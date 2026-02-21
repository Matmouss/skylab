import topography, graphics, os, json

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = r"config.json"

    config = json.load(open(config_json_path))

    current_map = topography.Map(config)

    print(current_map.get_elevation(3451535.0,2752425.0))
