cmake_minimum_required(VERSION 3.24)
project(waterlinked_dvl_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(libwaterlinked REQUIRED)
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
target_compile_features(waterlinked_dvl_driver_library PUBLIC cxx_std_20)
target_link_libraries(
    waterlinked_dvl_driver_library
    PRIVATE waterlinked_dvl_driver_parameters
    PUBLIC
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        rclcpp_components::component
        rclcpp_components::component_manager
        libwaterlinked::libwaterlinked
        ${geometry_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${marine_acoustic_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
)

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
ament_export_dependencies(
    "rclcpp"
    "rclcpp_lifecycle"
    "geometry_msgs"
    "std_srvs"
    "marine_acoustic_msgs"
    "nav_msgs"
    "tf2_geometry_msgs"
    "rclcpp_components"
    "libwaterlinked"
)

ament_package()
