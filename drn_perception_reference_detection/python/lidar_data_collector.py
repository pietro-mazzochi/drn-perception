import subprocess
import time
import threading
import signal
import os

package = "drc-simulation"
launch_file = "simulation.launch.py"
world_name="referencias"

def launch_gazebo(package, launch_file, params):
    command = f'ros2 launch {package} {launch_file} {params}'
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print('Gazebo started')
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
    process = launch_gazebo(package, launch_file, params)
    terminate_experiment.wait()
    process.terminate()

def record_thread(name, topic):
    process = start_record(name, topic)
    terminate_experiment.wait()
    process.terminate()

topic = "front_lidar"

padroes = {'padrao01': {'y': 0.0},
           'padrao02': {'y': 0.0},
           'padrao03': {'y': 0.0},
           'padrao04': {'y': 0.0},
           'padrao05': {'y': 0.0},
           'padrao06': {'y': 0.0},
           'padrao07': {'y': 0.0},
           'padrao08': {'y': 0.0},
           'padrao09': {'y': 0.0},
           'padrao10': {'y': 0.0},
           'padrao11': {'y': 0.0},
           'padrao12': {'y': 0.0},
           'padrao13': {'y': 0.0},
           'padrao14': {'y': 0.0},
           }
list_padroes = list(padroes.keys())

names = [
    # 'lidar_fixo_6',
    # 'lidar_fixo_5',
    'lidar_fixo_4',
    'lidar_fixo_3',
    # 'lidar_fixo_2',
    # 'lidar_fixo_1',
    # 'lidar_fixo_0'
    ]
xs = [
    # 4,
    # 5,
    6,
    7,
    # 8,
    # 9,
    # 9.5
]
y=0.0

padrao = 9

for x, name in zip(xs, names):
    terminate_experiment = threading.Event()

    gz_thread = threading.Thread(target=gazebo_thread, args=(package, launch_file, f'world_name:=v2_{list_padroes[padrao]} x:={x} y:={y} direction:=front'))
    # gazebo_thread(package, launch_file, f'world_name:={world_name} x:={x} y:={y}')
    gz_thread.start()
    time.sleep(20)

    exp_thread = threading.Thread(target=record_thread, args=(f'{list_padroes[padrao-1]}/{name}', topic))
    exp_thread.start()

    time.sleep(60)
    terminate_experiment.set()

    killem_all()
    print('30s to finish script')
    time.sleep(30)
