import subprocess
import time
import threading
import signal
import os
import yaml

lidar_to_base_x = 0.9135

package = "drn_perception_reference_detection"
launch_file = "experiment_launch.py"
world_name="referencias"

def build_workspace(ws_path):
    build_command = f'colcon build --packages-select {package}'
    subprocess.run(build_command, cwd=ws_path, shell=True, check=True)

def launch(package, launch_file, params):
    command = f'ros2 launch {package} {launch_file} {params}'
    # print(command)
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return process

def start_record(name, topic):
    command = f'ros2 bag record -o ~/ros_ws/src/drn-perception/drn_perception_reference_detection/experiments/data/{name} /{topic}'
    print(f'Recording experiment {command}')
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return process

def killem_all():
    # seek and destroy
    subprocess.run(['pkill', '-f', 'gzserver'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    subprocess.run(['pkill', '-f', 'gzclient'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    subprocess.run(['pkill', '-f', 'ros2 bag record'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    subprocess.run(['pkill', '-f', 'ros-args'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(5) 
    try:
        processes = subprocess.check_output(['pgrep', '-f', 'gzserver'], text=True).strip().split('\n')
        for pid in processes:
            print(f"Killing process with PID: {pid}")
            subprocess.run(['kill', '-9', pid])
    except:
        pass

def gazebo_thread(package, launch_file, params):
    print('Gazebo started')
    process = launch(package, launch_file, params)
    experiment_flag.wait()
    process.terminate()


def record_thread(name, topic):
    process = start_record(name, topic)
    experiment_flag.wait()
    process.terminate()

topic_pose = "drn_perception/dock_detector/service_response_topic"
topic_lidar = "drn_perception/dock_detector/debug"

# impar : reference_3d_height: float=0.2
# par : reference_3d_height: float=0.15
padroes = {'padrao01': {'reference': "[[0, -0.35], [0, -0.15], [0.15, 0], [0, 0.15], [0, 0.35]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.5,
                        'cluster_distance_threshold': 0.05,
                        'cluster_size': [0.2, 0.2],
                        'cluster_size_threshold': [0.02, 0.02],
                        'filter_radius': 0.385,
                        'filter_z_offset': 0.08,
                        'world_name': 'padrao_v_g'},
           'padrao02': {'reference': "[[0, -0.35], [0, -0.15], [0.15, 0], [0, 0.15], [0, 0.35]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.5,
                        'cluster_distance_threshold': 0.05,
                        'cluster_size': [0.2, 0.15],
                        'cluster_size_threshold': [0.02, 0.015],
                        'filter_radius': 0.385,
                        'filter_z_offset': 0.06,},
           'padrao03': {'reference': "[[0, -0.30], [0, -0.12], [0.12, 0], [0, 0.12], [0, 0.30]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.42,
                        'cluster_distance_threshold': 0.042,
                        'cluster_size': [0.18, 0.2],
                        'cluster_size_threshold': [0.018, 0.02],
                        'filter_radius': 0.33,
                        'filter_z_offset': 0.08,},
           'padrao04': {'reference': "[[0, -0.30], [0, -0.12], [0.12, 0], [0, 0.12], [0, 0.30]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.42,
                        'cluster_distance_threshold': 0.042,
                        'cluster_size': [0.18, 0.15],
                        'cluster_size_threshold': [0.018, 0.015],
                        'filter_radius': 0.33,
                        'filter_z_offset': 0.06,},
           'padrao05': {'reference': "[[0, -0.25], [0, -0.10], [0.10, 0], [0, 0.10], [0, 0.25]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.35,
                        'cluster_distance_threshold': 0.035,
                        'cluster_size': [0.15, 0.2],
                        'cluster_size_threshold': [0.015, 0.02],
                        'filter_radius': 0.275,
                        'filter_z_offset': 0.08,},
           'padrao06': {'reference': "[[0, -0.25], [0, -0.10], [0.10, 0], [0, 0.10], [0, 0.25]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.35,
                        'cluster_distance_threshold': 0.035,
                        'cluster_size': [0.15, 0.15],
                        'cluster_size_threshold': [0.015, 0.015],
                        'filter_radius': 0.275,
                        'filter_z_offset': 0.06,
                        'world_name': 'padrao_v_m'},
           'padrao07': {'reference': "[[0, -0.20], [0, -0.08], [0.08, 0], [0, 0.08], [0, 0.20]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.28,
                        'cluster_distance_threshold': 0.028,
                        'cluster_size': [0.12, 0.2],
                        'cluster_size_threshold': [0.012, 0.02],
                        'filter_radius': 0.22,
                        'filter_z_offset': 0.08,},
           'padrao08': {'reference': "[[0, -0.20], [0, -0.08], [0.08, 0], [0, 0.08], [0, 0.20]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.28,
                        'cluster_distance_threshold': 0.028,
                        'cluster_size': [0.12, 0.15],
                        'cluster_size_threshold': [0.012, 0.015],
                        'filter_radius': 0.22,
                        'filter_z_offset': 0.06,},
           'padrao09': {'reference': "[[0, -0.15], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.15]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.2,
                        'cluster_distance_threshold': 0.02,
                        'cluster_size': [0.1, 0.2],
                        'cluster_size_threshold': [0.01, 0.02],
                        'filter_radius': 0.165,
                        'filter_z_offset': 0.08,},
           'padrao10': {'reference': "[[0, -0.15], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.15]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.2,
                        'cluster_distance_threshold': 0.02,
                        'cluster_size': [0.1, 0.15],
                        'cluster_size_threshold': [0.01, 0.015],
                        'filter_radius': 0.165,
                        'filter_z_offset': 0.06,},
           'padrao11': {'reference': "[[0, -0.35], [0, 0.35]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.5,
                        'cluster_distance_threshold': 0.05,
                        'cluster_size': [0.2, 0.15],
                        'cluster_size_threshold': [0.02, 0.015],
                        'filter_radius': 0.385,
                        'filter_z_offset': 0.06,},
           'padrao12': {'reference': "[[0, -0.3], [0, 0.3]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.42,
                        'cluster_distance_threshold': 0.042,
                        'cluster_size': [0.18, 0.15],
                        'cluster_size_threshold': [0.018, 0.015],
                        'filter_radius': 0.33,
                        'filter_z_offset': 0.06,},
           'padrao13': {'reference': "[[0, -0.25], [0, 0.25]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.35,
                        'cluster_distance_threshold': 0.035,
                        'cluster_size': [0.15, 0.15],
                        'cluster_size_threshold': [0.015, 0.015],
                        'filter_radius': 0.275,
                        'filter_z_offset': 0.06,
                        'world_name': 'padrao_r_m'},
           'padrao14': {'reference': "[[0, -0.2], [0, 0.2]]",
                        'reference_3d_height': 0.15,
                        'cluster_distance': 0.28,
                        'cluster_distance_threshold': 0.028,
                        'cluster_size': [0.12, 0.15],
                        'cluster_size_threshold': [0.012, 0.05],
                        'filter_radius': 0.22,
                        'filter_z_offset': 0.06,},
            'padrao15': {'reference': "[[0, -0.35], [0, 0.35]]",
                        'reference_3d_height': 0.2,
                        'cluster_distance': 0.5,
                        'cluster_distance_threshold': 0.05,
                        'cluster_size': [0.2, 0.2],
                        'cluster_size_threshold': [0.02, 0.02],
                        'filter_radius': 0.385,
                        'filter_z_offset': 0.08,
                        'world_name': 'padrao_r_g'},
            'padrao16': {'reference': "[[0, -0.15], [0, 0.15]]",
                        'reference_3d_height': 0.1,
                        'cluster_distance': 0.2,
                        'cluster_distance_threshold': 0.02,
                        'cluster_size': [0.1, 0.1],
                        'cluster_size_threshold': [0.01, 0.01],
                        'filter_radius': 0.165,
                        'filter_z_offset': 0.04,
                        'world_name': 'padrao_v_p'},
            'padrao17': {'reference': "[[0, -0.15], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.15]]",
                        'reference_3d_height': 0.1,
                        'cluster_distance': 0.2,
                        'cluster_distance_threshold': 0.02,
                        'cluster_size': [0.1, 0.1],
                        'cluster_size_threshold': [0.01, 0.01],
                        'filter_radius': 0.165,
                        'filter_z_offset': 0.04,
                        'world_name': 'padrao_r_p'},
           }
list_padroes = list(padroes.keys())

names = [
    # 'fixo_0',
    # 'fixo_1',
    # 'fixo_2',
    'fixo_3',
    'fixo_4',
    'fixo_5',
    'fixo_6',    
        ]
xs = [
    # 9.5,
    # 9.0,
    # 8.0,
    7.0,
    6.0,
    5.0,
    4.0,
    ]


params_file = r"/home/allgayer/ros_ws/src/drn-perception/drn_perception_reference_detection/config/dock_detector_params.yaml"

# for padrao in range(0,14):
for padrao in [0,5,12,14,15,16]:
# for padrao in [15]:
    with open(params_file, 'r+') as file:
        content = yaml.safe_load(file)
        params = content['drn_perception']['dock_detector']['dock_detector_service_node']['ros__parameters']
        params['reference_3d_height'] = padroes[list_padroes[padrao]]["reference_3d_height"]
        params['reference'] = padroes[list_padroes[padrao]]["reference"]
        params['cluster_distance'] = padroes[list_padroes[padrao]]["cluster_distance"]
        params['cluster_distance_threshold'] = padroes[list_padroes[padrao]]["cluster_distance_threshold"]
        params['cluster_size'] = padroes[list_padroes[padrao]]["cluster_size"]
        params['cluster_size_threshold'] = padroes[list_padroes[padrao]]["cluster_size_threshold"]
        params['filter_radius'] = padroes[list_padroes[padrao]]["filter_radius"]
        params['filter_z_offset'] = padroes[list_padroes[padrao]]["filter_z_offset"]
        file.seek(0)
        yaml.dump(content, file, default_flow_style=False)
        file.truncate()

    build_workspace(os.path.expanduser('~/ros_ws'))

    for x, name in zip(xs, names):
        experiment_flag = threading.Event()
        # print(f'world_name:={list_padroes[padrao]} x:={x * (sinal)} y:={padroes[list_padroes[padrao]]["y"]} direction:=front')
        gz_thread = threading.Thread(target=gazebo_thread, args=(package, launch_file, f'world_name:={padroes[list_padroes[padrao]]["world_name"]} x:={x-lidar_to_base_x} y:={0.0} direction:=front debug:=True'))
        # gazebo_thread(package, launch_file, f'world_name:={world_name} x:={x} y:={y}')
        gz_thread.start()
        time.sleep(20)
        
        # collect_lidar_thread = threading.Thread(target=record_thread, args=(f'v4/{list_padroes[padrao]}/{name}', f'{topic_lidar} /{topic_pose}'))
        collect_lidar_thread = threading.Thread(target=record_thread, args=(f'v4/{padroes[list_padroes[padrao]]["world_name"]}/{name}', f'{topic_lidar} /{topic_pose}'))
        collect_lidar_thread.start()

        # collect_pose_thread = threading.Thread(target=record_thread, args=(f'v2/{list_padroes[padrao]}/pose_{name}', topic_pose))
        # collect_pose_thread.start()

        time.sleep(150)
        experiment_flag.set()

        killem_all()
        print('30s to finish script')
        time.sleep(30)