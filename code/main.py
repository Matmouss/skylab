import topography, fly_control, os, json, test

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    config_json_path = r"config.json"

    config = json.load(open(config_json_path))

    current_map = topography.Map(config)

    #test.carte_et_points(current_map, 10000)