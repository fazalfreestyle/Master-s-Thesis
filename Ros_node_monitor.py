import time
import threading
import roslibpy
import subprocess
import platform

# Whitelisted node prefixes (accounting for dynamic names)
whitelisted_prefixes = [
    "/controller_spawner",
    "/foxglove_bridge",
    "/foxglove_nodelet_manager",
    "/foxglove_spawner",
    "/move_group",
    "/move_group_commander_wrappers_1743451529236339445",
    "/move_group_commander_wrappers_1743451138642023486",
    "/niryo_robot_arm_commander",
    "/niryo_robot_database",
    "/niryo_robot_hardware_interface",
    "/niryo_robot_led_ring",
    "/niryo_robot_metrics",
    "/niryo_robot_modbus",
    "/niryo_robot_poses_handlers",
    "/niryo_robot_programs_manager",
    "/niryo_robot_programs_manager_v2",
    "/niryo_robot_reports",
    "/niryo_robot_rpi",
    "/niryo_robot_sound",
    "/niryo_robot_status",
    "/niryo_robot_tools_commander",
    "/niryo_robot_user_interface",
    "/niryo_robot_vision",
    "/robot_state_publisher",
    "/rosapi",
    "/rosbridge_websocket",
    "/rosout",
    "/tf2_web_republisher",
    "/robot1control",
    "/robot2control",
    "/matlab_introspec_31032025192149756"
]

def is_authorized(node_name):
    return any(node_name.startswith(prefix) for prefix in whitelisted_prefixes)

def check_topic_health(robot_ip):
    topic_name = '/niryo_robot_follow_joint_trajectory_controller/state'
    try:
        cmd = [
            'wsl', 'bash', '-c',
            f'export ROS_MASTER_URI=http://{robot_ip}:11311 && '
            'source /opt/ros/noetic/setup.bash && '
            f'rostopic info {topic_name}'
        ] if platform.system() == 'Windows' else [
            'bash', '-c',
            f'export ROS_MASTER_URI=http://{robot_ip}:11311 && '
            'source /opt/ros/noetic/setup.bash && '
            f'rostopic info {topic_name}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return '/niryo_robot_hardware_interface' in result.stdout
    except Exception as e:
        print(f"[⚠️] Failed to check topic health on {robot_ip}: {e}")
        return False

def revive_hardware_interface(robot_ip):
    print(f"[🔁] Attempting to fully relaunch Niryo ROS stack on {robot_ip}...")
    try:
        relaunch_cmd = (
            f'export ROS_MASTER_URI=http://{robot_ip}:11311 && '
            'source /opt/ros/noetic/setup.bash && '
            'roslaunch niryo_robot_bringup bringup.launch'
        )
        shell_cmd = ['wsl', 'bash', '-c', relaunch_cmd] if platform.system() == 'Windows' else ['bash', '-c', relaunch_cmd]
        subprocess.Popen(shell_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"[✅] Relaunch command sent to {robot_ip}")
    except Exception as e:
        print(f"[❌] Failed to relaunch robot stack on {robot_ip}: {e}")

def kill_node(node_name, robot_ip):
    print(f"killing {node_name}")
    try:
        cmd = [
            'wsl', 'bash', '-c',
            f'export ROS_MASTER_URI=http://{robot_ip}:11311 && '
            'source /opt/ros/noetic/setup.bash && '
            f'rosnode kill {node_name}'
        ] if platform.system() == 'Windows' else [
            'bash', '-c',
            f'export ROS_MASTER_URI=http://{robot_ip}:11311 && '
            'source /opt/ros/noetic/setup.bash && '
            f'rosnode kill {node_name}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        print(result.stdout.strip())
        print(f"[💀] Killed unauthorized node: {node_name}")
    except subprocess.TimeoutExpired:
        print(f"[⏳] Timeout while trying to kill node: {node_name}")
    except Exception as e:
        print(f"[❌] Failed to kill node {node_name}: {e}")

def monitor_robot(name, ip_address):
    print(f"[🔌] Connecting to {name} at {ip_address}...")
    client = roslibpy.Ros(host=ip_address, port=9090)
    client.run()

    if not client.is_connected:
        print(f"[❌] Failed to connect to {name}")
        return

    print(f"[✅] Connected to {name}")
    service = roslibpy.Service(client, '/rosapi/nodes', 'rosapi/Nodes')

    try:
        while client.is_connected:
            if not check_topic_health(ip_address):
                print(f"[🛑] Topic /niryo_robot_follow_joint_trajectory_controller/state not healthy on {name}!")
                revive_hardware_interface(ip_address)

            response = service.call(roslibpy.ServiceRequest())
            nodes = response['nodes']

            for node in nodes:
                if not is_authorized(node):
                    print(f"[🚨] Unauthorized node detected on {name}: {node}")
                    kill_node(node, ip_address)

            time.sleep(2)
    except Exception as e:
        print(f"[⚠️] Error while monitoring {name}: {e}")

    client.terminate()
    print(f"[⚠️] Disconnected from {name}")

# Robot IPs
robot1_ip = '192.168.47.152'
robot2_ip = '192.168.47.190'

# Launch monitoring threads
threading.Thread(target=monitor_robot, args=("Robot 1", robot1_ip)).start()
threading.Thread(target=monitor_robot, args=("Robot 2", robot2_ip)).start()
