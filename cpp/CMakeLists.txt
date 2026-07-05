cmake_minimum_required(VERSION 3.20)
project(routing3d_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/utf-8)
endif()

option(USE_OPENVDB "Enable OpenVDB-backed occupancy in the core/C API" ON)
option(ROUTING3D_REQUIRE_OPENVDB "Fail configure when USE_OPENVDB is ON but OpenVDB is unavailable" ON)
option(USE_FCL "Build FCL precise collision (needs vcpkg fcl)" OFF)
option(BUILD_PYTHON_BINDINGS "Build pybind11 module routing3d_cpp" OFF)

if(USE_OPENVDB)
    find_package(OpenVDB CONFIG QUIET)
    if(NOT OpenVDB_FOUND)
        if(ROUTING3D_REQUIRE_OPENVDB)
            message(FATAL_ERROR "OpenVDB is required. Install vcpkg package 'openvdb' and configure with -DCMAKE_TOOLCHAIN_FILE=<VCPKG_ROOT>/scripts/buildsystems/vcpkg.cmake, or set -DROUTING3D_REQUIRE_OPENVDB=OFF for a legacy non-VDB build.")
        else()
            message(WARNING "OpenVDB was not found; building legacy Dense/Implicit occupancy backend only.")
            set(USE_OPENVDB OFF CACHE BOOL "Enable OpenVDB-backed occupancy in the core/C API" FORCE)
        endif()
    endif()
endif()

add_library(routing3d_core STATIC
    src/occupancy.cpp
    src/sparse_occupancy.cpp
    src/scene_io.cpp
    src/rubber_band_engine.cpp
)
target_include_directories(routing3d_core PUBLIC include)

if(USE_OPENVDB)
    target_sources(routing3d_core PRIVATE src/vdb_occupancy.cpp)
    target_link_libraries(routing3d_core PUBLIC OpenVDB::openvdb)
    target_compile_definitions(routing3d_core PUBLIC ROUTING3D_USE_OPENVDB)
endif()

if(MSVC)
    target_compile_options(routing3d_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(routing3d_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(routing3d_cli cli/routing3d_cli.cpp)
target_link_libraries(routing3d_cli PRIVATE routing3d_core)

add_library(routing3d_capi SHARED capi/routing3d_capi.cpp)
target_link_libraries(routing3d_capi PRIVATE routing3d_core)
target_include_directories(routing3d_capi PUBLIC include capi)
target_compile_definitions(routing3d_capi PRIVATE ROUTING3D_CAPI_EXPORTS)

enable_testing()

add_executable(test_golden tests/test_golden.cpp)
target_link_libraries(test_golden PRIVATE routing3d_core)
add_test(NAME golden COMMAND test_golden)

add_executable(test_scene_io tests/test_scene_io.cpp)
target_link_libraries(test_scene_io PRIVATE routing3d_core)
target_compile_definitions(test_scene_io PRIVATE
    SCENE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
add_test(NAME scene_io COMMAND test_scene_io)

add_executable(test_occupancy tests/test_occupancy.cpp)
target_link_libraries(test_occupancy PRIVATE routing3d_core)
add_test(NAME occupancy COMMAND test_occupancy)

add_executable(test_corridor tests/test_corridor.cpp)
target_link_libraries(test_corridor PRIVATE routing3d_core)
add_test(NAME corridor COMMAND test_corridor)

add_executable(test_implicit tests/test_implicit.cpp)
target_link_libraries(test_implicit PRIVATE routing3d_core)
add_test(NAME implicit COMMAND test_implicit)

add_executable(test_octree tests/test_octree.cpp)
target_link_libraries(test_octree PRIVATE routing3d_core)
target_compile_definitions(test_octree PRIVATE
    SCENE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
add_test(NAME octree COMMAND test_octree
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_ripup tests/test_ripup.cpp)
target_link_libraries(test_ripup PRIVATE routing3d_core)
add_test(NAME ripup COMMAND test_ripup)

add_executable(test_attract tests/test_attract.cpp)
target_link_libraries(test_attract PRIVATE routing3d_core)
add_test(NAME attract COMMAND test_attract)

add_executable(test_capi tests/test_capi.cpp)
target_link_libraries(test_capi PRIVATE routing3d_capi)
target_include_directories(test_capi PRIVATE capi)
if(USE_OPENVDB)
    target_compile_definitions(test_capi PRIVATE ROUTING3D_USE_OPENVDB)
endif()
add_test(NAME capi COMMAND test_capi)

add_executable(test_realdata tests/test_realdata.cpp)
target_link_libraries(test_realdata PRIVATE routing3d_capi)
target_include_directories(test_realdata PRIVATE capi)
target_compile_definitions(test_realdata PRIVATE
    SCENE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
add_test(NAME realdata COMMAND test_realdata)

add_executable(test_category tests/test_category.cpp)
target_link_libraries(test_category PRIVATE routing3d_capi)
target_include_directories(test_category PRIVATE capi)
add_test(NAME category COMMAND test_category)

add_executable(test_rubber_band tests/test_rubber_band.cpp)
target_link_libraries(test_rubber_band PRIVATE routing3d_core)
add_test(NAME rubber_band COMMAND test_rubber_band)


if(USE_OPENVDB)
    add_executable(test_vdb tests/test_vdb.cpp)
    target_link_libraries(test_vdb PRIVATE routing3d_core)
    add_test(NAME vdb COMMAND test_vdb)
endif()

if(USE_FCL)
    find_package(fcl CONFIG REQUIRED)
    add_library(routing3d_fcl STATIC src/fcl_scene.cpp)
    target_include_directories(routing3d_fcl PUBLIC include)
    target_link_libraries(routing3d_fcl PUBLIC routing3d_core fcl)
    target_compile_definitions(routing3d_fcl PUBLIC ROUTING3D_USE_FCL)

    add_executable(test_fcl tests/test_fcl.cpp)
    target_link_libraries(test_fcl PRIVATE routing3d_fcl)
    add_test(NAME fcl COMMAND test_fcl)
endif()

if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(routing3d_cpp bindings/bindings.cpp)
    target_link_libraries(routing3d_cpp PRIVATE routing3d_core)

    set(ROUTING3D_TEST_PYTHON "${PYTHON_EXECUTABLE}" CACHE FILEPATH
        "Python interpreter for binding cross-validation (needs routing3d_py installed)")
    add_test(NAME bindings
        COMMAND "${ROUTING3D_TEST_PYTHON}"
                "${CMAKE_CURRENT_SOURCE_DIR}/bindings/test_bindings.py"
                "$<TARGET_FILE_DIR:routing3d_cpp>")
    # Windows: routing3d_cpp.pyd 의 전이 DLL(OpenVDB 등 vcpkg 런타임)이 PATH 에 없으면
    # ImportError: DLL load failed 가 발생한다. 테스트 실행 시 출력 디렉토리를 PATH 앞에 추가.
    if(WIN32)
        set_tests_properties(bindings PROPERTIES
            ENVIRONMENT "PATH=$<TARGET_FILE_DIR:routing3d_cpp>;$<TARGET_FILE_DIR:routing3d_capi>;$ENV{PATH}"
        )
    endif()
endif()
