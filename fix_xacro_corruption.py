import os
import re

files_to_fix = [
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/robotiq_2f_140/urdf/robotiq_2f_140_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f.ros2_control.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/robots/kortex_robot.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/arms/gen3/6dof/urdf/gen3_macro.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/arms/gen3/6dof/urdf/kortex.ros2_control.xacro"
]

for file_path in files_to_fix:
    if not os.path.exists(file_path):
        continue
    
    with open(file_path, "r") as f:
        lines = f.readlines()
    
    new_lines = []
    for line in lines:
        # Check for the corrupted line pattern
        if re.match(r'^\s+\)"\s*$', line):
            continue
        # Also check for partial corruption at end of lines
        cleaned_line = re.sub(r'\s+sim_gazebo\)"', '', line)
        cleaned_line = re.sub(r'\s+sim_isaac\)"', '', cleaned_line)
        cleaned_line = re.sub(r'\s+mock_sensor_commands\)"', '', cleaned_line)
        cleaned_line = re.sub(r'\s+isaac_joint_commands\)"', '', cleaned_line)
        cleaned_line = re.sub(r'\s+isaac_joint_states\)"', '', cleaned_line)
        
        # Another pattern seen: ' )"'
        if cleaned_line.strip() == ')"':
            continue
            
        new_lines.append(cleaned_line)
    
    if len(new_lines) != len(lines) or any(l1 != l2 for l1, l2 in zip(lines, new_lines)):
        with open(file_path, "w") as f:
            f.writelines(new_lines)
        print(f"Fixed {file_path}")
    else:
        print(f"No corruption found in {file_path}")
