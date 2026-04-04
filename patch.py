import sys

file_path = "/home/oscar/kinova_ws/src/ros2_kortex_vision/src/vision.cpp"
with open(file_path, "r") as f:
    text = f.read()

target1 = "node_->declare_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM);"
replacement1 = """if (!node_->has_parameter(CAMERA_INFO_URL_USER_PARAM)) {
    node_->declare_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM);
  }"""

target2 = "node_->declare_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM);"
replacement2 = """if (!node_->has_parameter(CAMERA_INFO_URL_DEFAULT_PARAM)) {
    node_->declare_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM);
  }"""

text = text.replace(target1, replacement1)
text = text.replace(target2, replacement2)

with open(file_path, "w") as f:
    f.write(text)
print("File patched.")
