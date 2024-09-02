cmake_minimum_required(VERSION 3.8)
project(waterlinked_driver)

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
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(libwaterlinked REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(waterlinked_driver_parameters
    src/waterlinked_driver_parameters.yaml
)

add_executable(waterlinked_driver)
target_sources(waterlinked_driver PRIVATE src/waterlinked_driver.cpp)

target_include_directories(
    waterlinked_driver
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(waterlinked_driver PUBLIC cxx_std_20)
target_link_libraries(
    waterlinked_driver
    PRIVATE waterlinked_driver_parameters libwaterlinked::libwaterlinked
)
ament_target_dependencies(waterlinked_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
    TARGETS waterlinked_driver waterlinked_driver_parameters
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
