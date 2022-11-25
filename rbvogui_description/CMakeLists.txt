cmake_minimum_required(VERSION 3.5)
project(rbvogui_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes robot test urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    rbvogui_urdf_xacro
    test/test_urdf_xacro.py
    TIMEOUT 30
  )

endif()

ament_package()
