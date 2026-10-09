import subprocess
import time
import threading
import signal
import os
import yaml

package = "drn_perception_reference_detection"
launch_file = "experiment_launch.py"
world_name="referencias"

def launch(package, launch_file, params):
    command = f'ros2 launch {package} {launch_file} {params}'
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
    terminate_experiment.wait()
    process.terminate()


def record_thread(name, topic):
    process = start_record(name, topic)
    terminate_experiment.wait()
    process.terminate()

topic = "drn_perception/dock_detector/service_response_topic"

# impar : reference_3d_height: float=0.2
# par : reference_3d_height: float=0.15
padroes = {'padrao01': {'y': -18.0, 'reference': "[[0, -0.35], [0, -0.15], [0.15, 0], [0, 0.15], [0, 0.35]]", 'reference_3d_height': 0.2},
           'padrao02': {'y': -18.0, 'reference': "[[0, -0.35], [0, -0.15], [0.15, 0], [0, 0.15], [0, 0.35]]", 'reference_3d_height': 0.15},
           'padrao03': {'y': -12.0, 'reference': "[[0, -0.30], [0, -0.12], [0.12, 0], [0, 0.12], [0, 0.30]]", 'reference_3d_height': 0.2},
           'padrao04': {'y': -12.0, 'reference': "[[0, -0.30], [0, -0.12], [0.12, 0], [0, 0.12], [0, 0.30]]", 'reference_3d_height': 0.15},
           'padrao05': {'y': -6.0, 'reference': "[[0, -0.25], [0, -0.10], [0.10, 0], [0, 0.10], [0, 0.25]]", 'reference_3d_height': 0.2},
           'padrao06': {'y': -6.0, 'reference': "[[0, -0.25], [0, -0.10], [0.10, 0], [0, 0.10], [0, 0.25]]", 'reference_3d_height': 0.15},
           'padrao07': {'y': 0.0, 'reference': "[[0, -0.20], [0, -0.08], [0.08, 0], [0, 0.08], [0, 0.20]]", 'reference_3d_height': 0.2},
           'padrao08': {'y': 0.0, 'reference': "[[0, -0.20], [0, -0.08], [0.08, 0], [0, 0.08], [0, 0.20]]", 'reference_3d_height': 0.15},
           'padrao09': {'y': 6.0, 'reference': "[[0, -0.10], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.10]]", 'reference_3d_height': 0.2},
           'padrao10': {'y': 6.0, 'reference': "[[0, -0.10], [0, -0.05], [0.05, 0], [0, 0.05], [0, 0.10]]", 'reference_3d_height': 0.15},
        #    'padrao11': {'y': 12.0, 'reference': ""},
        #    'padrao12': {'y': 12.0, 'reference': ""},
        #    'padrao13': {'y': 18.0, 'reference': ""},
        #    'padrao14': {'y': 18.0, 'reference': ""},
           }
list_padroes = list(padroes.keys())

names = [
    'pose_fixo_0',
    'pose_fixo_1',
    'pose_fixo_2',
    'pose_fixo_3',
    'pose_fixo_4',
    'pose_fixo_5',
    'pose_fixo_6',    
        ]
xs = [
    9.5,
    9.0,
    8.0,
    7.0,
    6.0,
    5.0,
    4.0,
    ]


params_file = r"/home/allgayer/ros_ws/src/drn-perception/drn_perception_reference_detection/config/dock_detector_params.yaml"

for padrao in range(8,10):
    with open(params_file, 'r+') as file:
        content = yaml.safe_load(file)
        params = content['drn_perception']['dock_detector']['dock_detector_service_node']['ros__parameters']
        params['reference_3d_height'] = padroes[list_padroes[padrao]]["reference_3d_height"]
        params['reference'] = padroes[list_padroes[padrao]]["reference"]
        file.seek(0)
        yaml.dump(content, file, default_flow_style=False)
        file.truncate()

    for x, name in zip(xs, names):
        terminate_experiment = threading.Event()
        # print(f'world_name:={list_padroes[padrao]} x:={x * (sinal)} y:={padroes[list_padroes[padrao]]["y"]} direction:=front')
        gz_thread = threading.Thread(target=gazebo_thread, args=(package, launch_file, f'world_name:=v2_{list_padroes[padrao]} x:={x} y:={padroes[list_padroes[padrao]]["y"]} direction:=front'))
        # gazebo_thread(package, launch_file, f'world_name:={world_name} x:={x} y:={y}')
        gz_thread.start()
        time.sleep(40)
        
        collect_thread = threading.Thread(target=record_thread, args=(f'all/{list_padroes[padrao]}/{name}', topic))
        collect_thread.start()

        time.sleep(100)
        terminate_experiment.set()

        killem_all()
        print('60s to finish script')
        time.sleep(60)