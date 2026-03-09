import topography, fly_control, os, json, graphics
import numpy as np

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    os.chdir(BASE_DIR)
    
    config = json.load(open("config.json"))
    current_map = topography.Map(config)

    try:
        # 1. 初始化任务点
        print("Génération des points de mission...")
        point_A = current_map.get_random_valid_point() 
        # 生成 3 个初始目标点 [B, C, D]
        points_initiales = [current_map.get_random_valid_point() for _ in range(3)] 
        
        print("\n--- 1. Calcul du trajet INITIAL ---")
        path_aller_init, path_retour_init = fly_control.boucle_principale(
            current_map, point_A, points_initiales, penalty=0.5
        )
        
        data_avant = {
            'path_aller': path_aller_init,
            'path_retour': path_retour_init,
            'points_out': [point_A] + points_initiales
        }

        # --- 2. 模拟动态任务 (关键改进区) ---
        print("\n--- 2. Simulation DYNAMIQUE ---")
        target_B = points_initiales[0]
        try:
            # 找到路径中最接近 target_B 的索引
            idx_at_B = path_aller_init.index(target_B)
            drone_current_pos = path_aller_init[idx_at_B]
        except ValueError:
            # 如果因为浮点误差找不到，就用 target_B 
            drone_current_pos = target_B
            idx_at_B = 0 

        print(f"Le drone est officiellement au point B : {drone_current_pos}")

        # 模拟收到新点 NEW
        point_NEW = current_map.get_random_valid_point()
        print(f"Saisie d'un nouveau point NEW: {point_NEW}")

        # 核心逻辑：排除已访问的 B 点
        remaining_targets = points_initiales[1:] + [point_NEW]
        
        # 重新规划从 B 出发到剩余点的路径
        path_aller_dyn_new_segments = []
        temp_pos = drone_current_pos
        targets_to_visit = remaining_targets.copy()
        
        while targets_to_visit:
            # 贪心选择最近的下一个点
            next_target = min(targets_to_visit, key=lambda t: fly_control.heuristic(
                current_map.convert_coords(*temp_pos), 
                current_map.convert_coords(*t)
            ))
            segment = fly_control.astar(current_map, temp_pos, next_target, 0.5)
            if segment:
                if not path_aller_dyn_new_segments: 
                    path_aller_dyn_new_segments.extend(segment)
                else: 
                    path_aller_dyn_new_segments.extend(segment[1:])
                temp_pos = next_target
                targets_to_visit.remove(next_target)
            else: 
                targets_to_visit.remove(next_target)

        # 重新计算回程路径
        path_retour_dyn = fly_control.astar(current_map, temp_pos, point_A, 0.5)

        # 拼接总轨迹：[已飞过的 A->B] + [新规划的 B->...]
        path_total_dyn = path_aller_init[:idx_at_B+1] + path_aller_dyn_new_segments[1:]
        
        data_apres = {
            'path_aller': path_total_dyn,
            'path_retour': path_retour_dyn,
            # 这里的 points_out 顺序：[当前位置(绿), 起点(蓝), 剩余目标(红)]
            'points_out': [drone_current_pos, point_A] + remaining_targets
        }

        # 3. 绘图
        print("\nAffichage du comparatif (Retour en NOIR)...")
        graphics.map_plot_compare(
            dataset=current_map.dataset, 
            map_shape=current_map.map_shape, 
            data_before=data_avant,
            data_after=data_apres
        )
            
    except Exception as e:
        print(f"Erreur lors de la mission : {e}")
        import traceback
        traceback.print_exc()