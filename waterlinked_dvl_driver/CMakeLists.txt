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
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(libwaterlinked REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(waterlinked_dvl_driver_parameters
    src/waterlinked_dvl_driver_parameters.yaml
)

add_executable(waterlinked_dvl_driver)
target_sources(waterlinked_dvl_driver PRIVATE src/waterlinked_dvl_driver.cpp)

target_compile_features(waterlinked_dvl_driver PUBLIC cxx_std_20)
target_link_libraries(
    waterlinked_dvl_driver
    PRIVATE waterlinked_dvl_driver_parameters libwaterlinked::libwaterlinked
)
ament_target_dependencies(waterlinked_dvl_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
    TARGETS waterlinked_dvl_driver waterlinked_dvl_driver_parameters
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/waterlinked_dvl_driver
)

install(DIRECTORY config launch DESTINATION share/waterlinked_dvl_driver)

ament_package()
