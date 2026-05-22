import os
import re

root = "/home/oscar/kinova_ws/src/ros2_kortex/kortex_description/"

for r, ds, fs in os.walk(root):
    for f in fs:
        if f.endswith(".xacro"):
            path = os.path.join(r, f)
            with open(path, "r") as file:
                lines = file.readlines()
                for i, line in enumerate(lines):
                    # Check for empty expressions ${}
                    if "${}" in line:
                        print(f"Empty expression: {path}:{i+1}: {line.strip()}")
                    # Check for empty value attributes value=""
                    if 'value=""' in line and '<xacro:if' in line:
                        print(f"Empty value in if: {path}:{i+1}: {line.strip()}")
                    # Check for leftover operators like 'or or' or 'and and'
                    if re.search(r'\s+or\s+or\s+', line) or re.search(r'\s+and\s+and\s+', line):
                        print(f"Double operator: {path}:{i+1}: {line.strip()}")
                    # Check for expressions starting with 'or ' or ending with ' or'
                    if re.search(r'\${\s*or\s+', line) or re.search(r'\s+or\s*}', line):
                        print(f"Dangling operator: {path}:{i+1}: {line.strip()}")
