import os
import re

files_to_patch = [
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/robotiq_2f_140/urdf/robotiq_2f_140_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f.ros2_control.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/robots/kortex_robot.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/arms/gen3/6dof/urdf/gen3_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/arms/gen3/6dof/urdf/kortex.ros2_control.xacro"
]

patterns = [
    # Remove assignments in calls: param="${value}" or param:=value
    r'\bsim_gazebo\s*[:=]\s*"\${[^}]*}"',
    r'\bsim_gazebo\s*[:=]\s*[^ \t\n>]+',
    r'\bsim_isaac\s*[:=]\s*"\${[^}]*}"',
    r'\bsim_isaac\s*[:=]\s*[^ \t\n>]+',
    r'\bmock_sensor_commands\s*[:=]\s*"\${[^}]*}"',
    r'\bmock_sensor_commands\s*[:=]\s*[^ \t\n>]+',
    r'\bisaac_joint_commands\s*[:=]\s*"\${[^}]*}"',
    r'\bisaac_joint_commands\s*[:=]\s*[^ \t\n>]+',
    r'\bisaac_joint_states\s*[:=]\s*"\${[^}]*}"',
    r'\bisaac_joint_states\s*[:=]\s*[^ \t\n>]+',
    # Remove in param tags
    r'<param\s+name="(?:sim_gazebo|sim_isaac|mock_sensor_commands|isaac_joint_commands|isaac_joint_states)">[^<]*</param>',
    # Remove in macro param definitions: param or param:=default
    r'\bsim_gazebo\s*(?::=\s*[^ \t\n,"]+)?',
    r'\bsim_isaac\s*(?::=\s*[^ \t\n,"]+)?',
    r'\bmock_sensor_commands\s*(?::=\s*[^ \t\n,"]+)?',
    r'\bisaac_joint_commands\s*(?::=\s*[^ \t\n,"]+)?',
    r'\bisaac_joint_states\s*(?::=\s*[^ \t\n,"]+)?',
]

for file_path in files_to_patch:
    if not os.path.exists(file_path):
        print(f"Skipping {file_path}")
        continue
    
    with open(file_path, "r") as f:
        content = f.read()
    
    new_content = content
    # Handle the 'or sim_gazebo' expressions first
    new_content = re.sub(r'\s+or\s+sim_gazebo', '', new_content)
    new_content = re.sub(r'sim_gazebo\s+or\s+', '', new_content)
    new_content = re.sub(r'\s+or\s+sim_isaac', '', new_content)
    new_content = re.sub(r'sim_isaac\s+or\s+', '', new_content)

    for pattern in patterns:
        new_content = re.sub(pattern, '', new_content)
    
    # Fix the internal bus default
    if "gen3.xacro" in file_path:
        new_content = re.sub(r'<xacro:arg\s+name="use_internal_bus_gripper_comm"\s+default="false"\s*/>', 
                             '<xacro:arg name="use_internal_bus_gripper_comm" default="true" />', new_content)

    if new_content != content:
        with open(file_path, "w") as f:
            f.write(new_content)
        print(f"Patched {file_path}")
    else:
        print(f"No changes needed for {file_path}")
