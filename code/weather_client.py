import requests, math

def get_wind(lat: float, lon: float) -> dict:
    """
    Interroge Open-Meteo pour obtenir le vent courant au point (lat, lon).
    Retourne : {'u': float, 'v': float, 'speed': float, 'dir_deg': float}
    u = composante Est (m/s), v = composante Nord (m/s)
    """
    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lat, "longitude": lon,
        "current": "wind_speed_10m,wind_direction_10m",
        "wind_speed_unit": "ms",
        "forecast_days": 1
    }
    try:
        r = requests.get(url, params=params, timeout=5)
        r.raise_for_status()
        curr = r.json()["current"]
        speed = curr["wind_speed_10m"]          # m/s
        dir_deg = curr["wind_direction_10m"]    # degrés météo (0=N, 90=E)
        dir_rad = math.radians(dir_deg)
        # Convention météo : le vent VIENT de dir_deg → vecteur va vers l'opposé
        u =  speed * math.sin(dir_rad)          # composante Est
        v =  speed * math.cos(dir_rad)          # composante Nord
        return {"u": u, "v": v, "speed": speed, "dir_deg": dir_deg}
    except Exception as e:
        print(f"[WeatherClient] Erreur API : {e} — vent nul utilisé")
        return {"u": 0.0, "v": 0.0, "speed": 0.0, "dir_deg": 0.0}