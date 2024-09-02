cmake_minimum_required(VERSION 3.8)
project(waterlinked_driver LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDENCIES
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  libwaterlinked
  geometry_msgs
  std_msgs
  marine_acoustic_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(waterlinked_driver_parameters
  src/waterlinked_driver_parameters.yaml
)

add_library(waterlinked_driver SHARED)
target_sources(waterlinked_driver PRIVATE src/waterlinked_driver.cpp)

target_include_directories(
    waterlinked_driver
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(waterlinked_driver PUBLIC cxx_std_20)
target_link_libraries(waterlinked_driver PRIVATE waterlinked_driver_parameters libwaterlinked)
ament_target_dependencies(waterlinked_driver ${THIS_PACKAGE_INCLUDE_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    TARGETS waterlinked_driver waterlinked_driver_parameters
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
