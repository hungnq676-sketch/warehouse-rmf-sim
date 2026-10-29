# Dung cho moi terminal lam viec voi kho + RMF:
#     source kho_env.sh
# ROS_LOCALHOST_ONLY=1: chi noi chuyen ROS trong may nay, tranh lan /clock, TF...
# voi may khac cung mang LAN (kho_rmf.launch.py cung dat nhu vay).
source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
