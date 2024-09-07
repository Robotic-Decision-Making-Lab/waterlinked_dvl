cmake_minimum_required(VERSION 3.24)
project(
    libwaterlinked
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "C++ library used to interact with the Water Linked DVL devices"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(libwaterlinked SHARED)

target_sources(
    libwaterlinked
    PRIVATE src/client.cpp src/protocol.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libwaterlinked/client.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libwaterlinked/protocol.hpp
)
target_compile_features(libwaterlinked PUBLIC cxx_std_20)
target_link_libraries(
    libwaterlinked
    PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json
)
set_target_properties(libwaterlinked PROPERTIES PREFIX "")

set(EXAMPLES send_command subscribe_to_reports)
foreach(example IN ITEMS ${EXAMPLES})
    add_executable(${example} examples/${example}.cpp)
    add_dependencies(${example} libwaterlinked)
    target_link_libraries(
        ${example}
        PUBLIC libwaterlinked
        PRIVATE nlohmann_json::nlohmann_json Eigen3::Eigen
    )
    set_target_properties(
        ${example}
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/examples
    )
endforeach()

install(
    TARGETS libwaterlinked
    EXPORT libwaterlinkedTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)

    ament_add_gtest(test_json test/test_json.cpp)
    target_link_libraries(
        test_json
        libwaterlinked
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
    )
endif()

ament_export_targets(libwaterlinkedTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 nlohmann_json)

ament_package()
