cmake_minimum_required(VERSION 3.8)
project(rubi_height_mapping)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/height_mapping_node scripts/check_environment DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE UPSTREAM.md README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(height_contract test/test_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(height_odometry test/test_odometry.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(height_export test/test_gridmap_export.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  option(RUBI_TEST_CUDA "Run GPU estimator and isolated ROS pipeline tests" OFF)
  if(RUBI_TEST_CUDA)
    ament_add_pytest_test(height_gpu test/test_gpu.py
      ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
    ament_add_pytest_test(height_ray_selection test/test_ray_selection.py
      ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
    ament_add_pytest_test(height_snapshot_fusion test/test_snapshot_fusion.py
      ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
    ament_add_pytest_test(height_ros_pipeline test/test_ros_pipeline.py
      ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 180)
    ament_add_pytest_test(height_ros_odometry test/test_ros_odometry.py
      ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 180)
  endif()
endif()
ament_package()
