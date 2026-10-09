import subprocess
import time
import threading
import os
import update_detection_params

lidar_to_base_x = 0

def build_workspace(ws_path):
    build_command = f'colcon build --packages-select drn_perception_reference_detection'
    subprocess.run(build_command, cwd=ws_path, shell=True, check=True)

def launch(package, launch_file, params):
    command = f'ros2 launch {package} {launch_file} {params}'
    # print(command)
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return process

def start_record(name, topic):
    command = f'ros2 bag record -o ~/ros_ws/src/drn-perception/drn_perception_reference_detection/experiments/data/{name} -d=10 /{topic}'
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

def experiment_thread(package, launch_file, params):
    print('experiment started')
    process = launch(package, launch_file, params)
    experiment_flag.wait()
    process.terminate()

def record_thread(name, topic):
    process = start_record(name, topic)
    experiment_flag.wait()
    process.terminate()

topic_pose = "drn_perception/dock_detector/service_response_topic"
topic_lidar = "drn_perception/dock_detector/debug"

names = [
    # 'fixo_0',
    # 'fixo_1',
    'fixo_2',
    'fixo_3',
    'fixo_4',
    'fixo_5',
    'fixo_6',    
        ]
xs = [
    # 9.5,
    # 9.0,
    8.0,
    7.0,
    6.0,
    5.0,
    4.0,
    ]

padroes = ['padrao_v_g', 'padrao_v_m', 'padrao_r_m', 'padrao_r_g', 'padrao_v_p', 'padrao_r_p']

for padrao in padroes:

    update_detection_params.main(padrao)
    print(f'Parâmetros atualizado para {padrao}')

    build_workspace(os.path.expanduser('~/ros_ws'))

    for x, name in zip(xs, names):
        experiment_flag = threading.Event()
        x=x-lidar_to_base_x
        print(f'world:={padrao} x:={x} y:={0.0}')
        gz_thread = threading.Thread(target=gazebo_thread, args=('drc_simulation', 'gazebo.launch.xml', f'world:={padrao} x:={x} y:={0.0} robot:=lidar_robot'))
        gz_thread.start()
        time.sleep(20)
        
        exp_thread = threading.Thread(target=experiment_thread, args=('drn_perception_reference_detection', 'experiment.launch.py', f'direction:=front debug:=True base_frame:=base_link'))
        exp_thread.start()
        time.sleep(15)
        
        collect_lidar_thread = threading.Thread(target=record_thread, args=(f'v10/{padrao}/{name}', f'{topic_lidar} /{topic_pose}'))
        collect_lidar_thread.start()

        time.sleep(60)
        experiment_flag.set()

        killem_all()
        print('60 to finish script')
        time.sleep(60)