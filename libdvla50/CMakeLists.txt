cmake_minimum_required(VERSION 3.8)
project(
    libdvla50
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "C++ library used to interact with the WaterLinked DVL-A50"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(libdvla50 SHARED)
target_include_directories(
    libdvla50
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(libdvla50 PUBLIC cxx_std_20)
target_link_libraries(libdvla50 PRIVATE nlohmann_json::nlohmann_json)
set_target_properties(libdvla50 PROPERTIES PREFIX "")

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    TARGETS libdvla50
    EXPORT libdvla50Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_export_targets(libdvla50Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)

ament_package()
