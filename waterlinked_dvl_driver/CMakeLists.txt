cmake_minimum_required(VERSION 3.8)
project(waterlinked_dvl_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    std_srvs
    marine_acoustic_msgs
    nav_msgs
    tf2_geometry_msgs
    rclcpp_components
    libwaterlinked
)

foreach(pkg IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${pkg} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(waterlinked_dvl_driver_parameters
    src/waterlinked_dvl_driver_parameters.yaml
)

add_library(waterlinked_dvl_driver_library SHARED)

target_sources(
    waterlinked_dvl_driver_library
    PRIVATE src/waterlinked_dvl_driver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/waterlinked_dvl_driver/waterlinked_dvl_driver.hpp
)
target_link_libraries(
    waterlinked_dvl_driver_library
    PRIVATE waterlinked_dvl_driver_parameters
)
ament_target_dependencies(waterlinked_dvl_driver_library PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(waterlinked_dvl_driver_library PUBLIC cxx_std_20)

rclcpp_components_register_node(waterlinked_dvl_driver_library
    PLUGIN "waterlinked::ros::WaterLinkedDvlDriver"
    EXECUTABLE waterlinked_dvl_driver
)

install(
    TARGETS waterlinked_dvl_driver_library waterlinked_dvl_driver_parameters
    EXPORT export_waterlinked_dvl_driver_library
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

install(DIRECTORY config launch DESTINATION share/waterlinked_dvl_driver)

ament_export_targets(export_waterlinked_dvl_driver_library HAS_LIBRARY_TARGET)
ament_export_dependencies($THIS_PACKAGE_INCLUDE_DEPENDS)

ament_package()
