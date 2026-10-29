cmake_minimum_required(VERSION 3.8)
project(warehouse_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds launch rviz config maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# Node Python chay bang `ros2 run warehouse_world <ten>.py`
install(
  PROGRAMS
    scripts/amr_fleet_driver.py
    scripts/kho_workcells.py
    scripts/dispatch_delivery.py
    scripts/make_map.py
    scripts/make_rmf_nav_graph.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
