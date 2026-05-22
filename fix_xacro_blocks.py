import os
import re

files_to_fix = [
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/arms/gen3/6dof/urdf/kortex.ros2_control.xacro",
    "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f.ros2_control.xacro"
]

def remove_broken_blocks(content):
    # Pattern to match <xacro:if value="${}"> ... </xacro:if>
    # or <xacro:unless value="${}"> ... </xacro:unless>
    # This is tricky with regex due to nested tags, but we can try a simple non-greedy match
    # since these specific blocks are usually simple.
    
    # First, handle the single lines like <param name="..."> ${} </param>
    content = re.sub(r'<param\s+name="[^"]*">\${}</param>', '', content)
    
    # Then handle the blocks
    # We search for <xacro:(if|unless) value="\${}"> and match until the matching </xacro:\1>
    # Since we know these specific blocks don't have much nesting, we can use a regex
    patterns = [
        r'<xacro:if\s+value="\${}">.*?</xacro:if>',
        r'<xacro:unless\s+value="\${}">.*?</xacro:unless>'
    ]
    
    for pattern in patterns:
        content = re.sub(pattern, '', content, flags=re.DOTALL)
        
    return content

for file_path in files_to_fix:
    if not os.path.exists(file_path):
        continue
    
    with open(file_path, "r") as f:
        content = f.read()
    
    new_content = remove_broken_blocks(content)
    
    if new_content != content:
        with open(file_path, "w") as f:
            f.write(new_content)
        print(f"Fixed blocks in {file_path}")
    else:
        print(f"No broken blocks found in {file_path}")
