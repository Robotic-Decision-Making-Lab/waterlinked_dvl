cmake_minimum_required(VERSION 3.8)
project(
    libwaterlinked
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "C++ library used to interact with the WaterLinked DVL devices"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(libwaterlinked SHARED)
target_sources(libwaterlinked PRIVATE src/client.cpp src/protocol.cpp)

target_include_directories(
    libwaterlinked
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(libwaterlinked PUBLIC cxx_std_20)
target_link_libraries(
    libwaterlinked
    PRIVATE nlohmann_json::nlohmann_json Eigen3::Eigen
)
set_target_properties(libwaterlinked PROPERTIES PREFIX "")

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    TARGETS libwaterlinked
    EXPORT libwaterlinkedTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_export_targets(libwaterlinkedTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json Eigen3)

ament_package()
