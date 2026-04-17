import math
import numpy as np

class DroneEnergyModel:
    """
    Modèle énergétique simplifié pour un drone multirotor.

    Hypothèses :
      - Vitesse air constante (cruise_speed)
      - Puissance de survol (hover) = P_hover
      - Puissance de croisière ≈ P_hover × facteur (typiquement 1.1–1.5)
      - Traînée aérodynamique proportionnelle à v_air²
    """
    def __init__(self, config: dict):
        self.mass_kg        = config.get("mass_kg", 2.0)
        self.battery_wh     = config.get("battery_wh", 200.0)
        self.cruise_speed   = config.get("cruise_speed_ms", 10.0)   # m/s
        self.hover_power_w  = config.get("hover_power_w", 150.0)    # Watts
        self.drag_coef      = config.get("drag_coef", 0.3)          # sans unité

        # Puissance de croisière (vol horizontal sans vent)
        # Formule approchée : P_cruise ≈ P_hover + 0.5 * Cd * v²
        # Le terme de traînée dépend du vent relatif dans segment_cost.

    def segment_energy_j(self, dist_m: float, wind: dict) -> float:
        """
        Énergie en Joules pour parcourir dist_m mètres dans la direction
        d'un segment, avec le vecteur vent donné.

        On suppose que le drone vole en cap direct (pas de dérive).
        La vitesse sol = cruise_speed + vent favorable (projection sur le cap).

        Retourne : (energie_joules, temps_secondes)
        """
        # Vecteur cap normalisé (on n'a que la distance ici, pas l'azimut)
        # → version simplifiée : projection scalaire du vent
        # Pour une version exacte, passer le vecteur direction du segment.
        wind_proj = wind.get("headwind", 0.0)   # > 0 = vent de face, < 0 = vent arrière

        v_air = self.cruise_speed + wind_proj   # vitesse air effective
        v_air = max(v_air, 1.0)                 # plancher de sécurité

        # Temps de vol du segment
        v_ground = self.cruise_speed - wind_proj
        v_ground = max(v_ground, 0.5)
        t_s = dist_m / v_ground

        # Puissance = hover + terme de traînée (v_air²)
        p_w = self.hover_power_w + 0.5 * self.drag_coef * (v_air ** 2)

        energy_j = p_w * t_s
        return energy_j, t_s

    def project_wind_on_segment(self, p1: tuple, p2: tuple, wind: dict) -> float:
        """
        Projette le vecteur vent sur l'axe du segment p1→p2.
        Retourne la composante de vent de face (positive = vent défavorable).
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0
        ux, uy = dx / dist, dy / dist           # vecteur unitaire du cap
        # Vent = (u=Est, v=Nord) → projeter sur (ux, uy)
        wind_along = wind["u"] * ux + wind["v"] * uy
        # Convention : vent de face = s'oppose au déplacement
        return -wind_along

    def total_path_energy(self, path: list, wind: dict) -> dict:
        """
        Calcule énergie et temps total d'un chemin (liste de (x,y)).
        """
        total_j = 0.0
        total_s = 0.0
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            hw = self.project_wind_on_segment(p1, p2, wind)
            e_j, t_s = self.segment_energy_j(dist, {"headwind": hw})
            total_j += e_j
            total_s += t_s
        battery_j = self.battery_wh * 3600
        return {
            "energy_j": total_j,
            "time_s":   total_s,
            "battery_%": min(100.0, total_j / battery_j * 100)
        }