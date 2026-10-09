import yaml
import sys
import os
from pathlib import Path

padroes = {
    'padrao_v_g': {'reference': "[[0, -0.35], [0, -0.15], [0.15, 0], [0, 0.15], [0, 0.35]]",
                'reference_3d_height': 0.2,
                'cluster_distance': 0.5,
                'cluster_distance_threshold': 0.05,
                'cluster_size': [0.2, 0.2],
                'cluster_size_threshold': [0.02, 0.04],
                'filter_radius': 0.385,
                'filter_z_offset': 0.08,
                'world_name': 'padrao_v_g'},
    'padrao_v_m': {'reference': "[[0, -0.25], [0, -0.10], [0.10, 0], [0, 0.10], [0, 0.25]]",
                'reference_3d_height': 0.15,
                'cluster_distance': 0.35,
                'cluster_distance_threshold': 0.035,
                'cluster_size': [0.15, 0.15],
                'cluster_size_threshold': [0.015, 0.03],
                'filter_radius': 0.275,
                'filter_z_offset': 0.06,
                'world_name': 'padrao_v_m'},
    'padrao_v_p': {'reference': "[[0, -0.15], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.15]]",
                'reference_3d_height': 0.1,
                'cluster_distance': 0.2,
                'cluster_distance_threshold': 0.02,
                'cluster_size': [0.1, 0.1],
                'cluster_size_threshold': [0.01, 0.02],
                'filter_radius': 0.165,
                'filter_z_offset': 0.04,
                'world_name': 'padrao_v_p'},
    'padrao_r_g': {'reference': "[[0, -0.35], [0, 0.35]]",
                'reference_3d_height': 0.2,
                'cluster_distance': 0.5,
                'cluster_distance_threshold': 0.05,
                'cluster_size': [0.2, 0.2],
                'cluster_size_threshold': [0.02, 0.04],
                'filter_radius': 0.385,
                'filter_z_offset': 0.08,
                'world_name': 'padrao_r_g'},
    'padrao_r_m': {'reference': "[[0, -0.25], [0, 0.25]]",
                'reference_3d_height': 0.15,
                'cluster_distance': 0.35,
                'cluster_distance_threshold': 0.035,
                'cluster_size': [0.15, 0.15],
                'cluster_size_threshold': [0.015, 0.03],
                'filter_radius': 0.275,
                'filter_z_offset': 0.06,
                'world_name': 'padrao_r_m'},
    'padrao_r_p': {'reference': "[[0, -0.15], [0, 0.15]]",
                'reference_3d_height': 0.1,
                'cluster_distance': 0.2,
                'cluster_distance_threshold': 0.02,
                'cluster_size': [0.1, 0.1],
                'cluster_size_threshold': [0.01, 0.02],
                'filter_radius': 0.165,
                'filter_z_offset': 0.04,
                'world_name': 'padrao_r_p'},
           }

script_dir = Path(__file__).resolve().parent
params_file = script_dir.parent / 'config' / 'dock_detector_params.yaml'

def main(padrao = 'padrao01'):
    with open(params_file, 'r+') as file:
            content = yaml.safe_load(file)
            params = content['drn_perception']['dock_detector']['dock_detector_service_node']['ros__parameters']
            params['reference_3d_height'] = padroes[padrao]["reference_3d_height"]
            params['reference'] = padroes[padrao]["reference"]
            params['cluster_distance'] = padroes[padrao]["cluster_distance"]
            params['cluster_distance_threshold'] = padroes[padrao]["cluster_distance_threshold"]
            params['cluster_size'] = padroes[padrao]["cluster_size"]
            params['cluster_size_threshold'] = padroes[padrao]["cluster_size_threshold"]
            params['filter_radius'] = padroes[padrao]["filter_radius"]
            params['filter_z_offset'] = padroes[padrao]["filter_z_offset"]
            file.seek(0)
            yaml.dump(content, file, default_flow_style=False)
            file.truncate()

if __name__ == "__main__":
    padrao = sys.argv[1] if len(sys.argv) > 1 else "padrao01"
    main(padrao)