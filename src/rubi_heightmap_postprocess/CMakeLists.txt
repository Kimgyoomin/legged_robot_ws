cmake_minimum_required(VERSION 3.8)
project(rubi_heightmap_postprocess)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/compare scripts/preview scripts/edit DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(postprocess_contract test/test_postprocess.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
  ament_add_pytest_test(repair_contract test/test_repair.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
  ament_add_pytest_test(editor_smoke test/test_editor.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 TIMEOUT 120)
endif()
ament_package()
