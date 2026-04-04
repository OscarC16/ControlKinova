import re
import os

filepath = "/home/oscar/kinova_ws/src/ros2_kortex/kortex_driver/src/hardware_interface.cpp"

with open(filepath, 'r') as f:
    content = f.read()

# Remove any previous SetMessageTimeout if it existed
content = content.replace('router_udp_realtime_.SetMessageTimeout(500);', '')
content = content.replace('router_udp_realtime_.SetMessageTimeout(1000);', '')

# Update options to 200ms
options = 'k_api::RouterClientSendOptions{false, 0, 200}'

# Using regex to update any Refresh calls that have the options struct
content = re.sub(
    r'k_api::RouterClientSendOptions\{false, 0, \d+\}',
    options,
    content
)

with open(filepath, 'w') as f:
    f.write(content)

print("Patch with 200ms explicit options applied.")
