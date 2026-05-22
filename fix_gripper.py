import os

filepath = "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro"
with open(filepath, 'r') as f:
    content = f.read()

# Search for the specific line and replace it
target = 'include_ros2_control="${include_ros2_control}"'
replacement = 'include_ros2_control="false"'

if target in content:
    content = content.replace(target, replacement)
    with open(filepath, 'w') as f:
        f.write(content)
    print("Replacement successful.")
else:
    print("Target string not found.")
