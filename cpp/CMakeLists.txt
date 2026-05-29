# Routing3D C++ 엔진 (Phase 3) — 빌드 스크립트
# =============================================================================
# [빌드/실행]  (프로젝트 루트에서)
#   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
#   cmake --build cpp/build --config Release
#   ctest --test-dir cpp/build -C Release --output-on-failure
#
# 현재는 의존성 없는 코어(occupancy / cost / astar)만 빌드한다.
# Boost.Heap / abseil / OpenVDB / FCL / pybind11 은 후속 단계(3.6~3.10)에서 vcpkg 로 추가.
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(routing3d_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 소스는 UTF-8(한글 주석). MSVC 가 코드페이지(예: CP949)로 오해해 줄을 삼키지 않도록 강제.
if(MSVC)
    add_compile_options(/utf-8)
endif()

# 엔진(astar/cost/multi_route)은 백엔드 무관 템플릿(헤더 정의) → .cpp 없음.
add_library(routing3d_core STATIC
    src/occupancy.cpp
    src/sparse_occupancy.cpp
    src/scene_io.cpp
)
target_include_directories(routing3d_core PUBLIC include)

if(MSVC)
    target_compile_options(routing3d_core PRIVATE /W4 /permissive-)
endif()

# 엔진 실행 CLI (scene.txt 라우팅 + 내장 데모). 외부 의존성 불필요(코어만).
add_executable(routing3d_cli cli/routing3d_cli.cpp)
target_link_libraries(routing3d_cli PRIVATE routing3d_core)

# 네이티브 C ABI 공유 라이브러리 (C#/P-Invoke·ctypes 인터롭). 코어만 링크 → 외부 의존성 없는 단일 DLL.
#   산출물: routing3d_capi.dll (+ import .lib). 설계: docs/csharp_helix_interop_design.md.
add_library(routing3d_capi SHARED capi/routing3d_capi.cpp)
target_link_libraries(routing3d_capi PRIVATE routing3d_core)
target_include_directories(routing3d_capi PUBLIC include capi)
target_compile_definitions(routing3d_capi PRIVATE ROUTING3D_CAPI_EXPORTS)

enable_testing()
add_executable(test_golden tests/test_golden.cpp)
target_link_libraries(test_golden PRIVATE routing3d_core)
add_test(NAME golden COMMAND test_golden)

# scene.txt I/O 교차검증(Python 픽스처를 읽어 바이트 동일 재출력). 픽스처 경로를 매크로로 전달.
add_executable(test_scene_io tests/test_scene_io.cpp)
target_link_libraries(test_scene_io PRIVATE routing3d_core)
target_compile_definitions(test_scene_io PRIVATE
    SCENE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
add_test(NAME scene_io COMMAND test_scene_io)

# 점유맵 백엔드(Dense vs Sparse) O1 동등성 + 8,000m 메모리 검증.
add_executable(test_occupancy tests/test_occupancy.cpp)
target_link_libraries(test_occupancy PRIVATE routing3d_core)
add_test(NAME occupancy COMMAND test_occupancy)

# 계층 corridor + 해시 A*: 거대 장면 로컬 배관 + coarse 가이드 tube 검증.
add_executable(test_corridor tests/test_corridor.cpp)
target_link_libraries(test_corridor PRIVATE routing3d_core)
add_test(NAME corridor COMMAND test_corridor)

# rip-up & reroute(Step 3.8): 합성 혼잡 시나리오에서 순차 1/2 → rip-up 2/2(Python 일치).
add_executable(test_ripup tests/test_ripup.cpp)
target_link_libraries(test_ripup PRIVATE routing3d_core)
add_test(NAME ripup COMMAND test_ripup)

# C ABI 스모크 테스트: DLL 경유 골든03 재현(5/5·28050mm) + 문자열 ABI 왕복.
add_executable(test_capi tests/test_capi.cpp)
target_link_libraries(test_capi PRIVATE routing3d_capi)
target_include_directories(test_capi PRIVATE capi)
add_test(NAME capi COMMAND test_capi)

# OpenVDB 백엔드(VdbOccupancy) — 기본 OFF. 활성화 시 vcpkg 의 OpenVDB 필요.
#   cmake ... -DUSE_OPENVDB=ON -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake \
#             -DVCPKG_TARGET_TRIPLET=x64-windows
option(USE_OPENVDB "Build OpenVDB-backed occupancy (needs vcpkg openvdb)" OFF)
if(USE_OPENVDB)
    find_package(OpenVDB CONFIG REQUIRED)
    add_library(routing3d_vdb STATIC src/vdb_occupancy.cpp)
    target_include_directories(routing3d_vdb PUBLIC include)
    target_link_libraries(routing3d_vdb PUBLIC routing3d_core OpenVDB::openvdb)
    target_compile_definitions(routing3d_vdb PUBLIC ROUTING3D_USE_OPENVDB)

    add_executable(test_vdb tests/test_vdb.cpp)
    target_link_libraries(test_vdb PRIVATE routing3d_vdb)
    add_test(NAME vdb COMMAND test_vdb)
endif()

# FCL 정밀 충돌(FclScene) — 기본 OFF. 활성화 시 vcpkg 의 fcl 필요.
#   cmake ... -DUSE_FCL=ON -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake \
#             -DVCPKG_TARGET_TRIPLET=x64-windows
option(USE_FCL "Build FCL precise collision (needs vcpkg fcl)" OFF)
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

# pybind11 모듈(routing3d_cpp) — 기본 OFF. 활성화 시 .venv 의 pybind11/Python 필요.
#   cmake ... -DBUILD_PYTHON_BINDINGS=ON -Dpybind11_DIR=<pybind11.get_cmake_dir()> \
#             -DROUTING3D_TEST_PYTHON=<.venv\Scripts\python.exe>
option(BUILD_PYTHON_BINDINGS "Build pybind11 module routing3d_cpp" OFF)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(routing3d_cpp bindings/bindings.cpp)
    target_link_libraries(routing3d_cpp PRIVATE routing3d_core)

    # 바인딩 교차검증 ctest. routing3d_py(editable) 가 설치된 파이썬이 필요(기본=빌드시 발견한 Python).
    set(ROUTING3D_TEST_PYTHON "${PYTHON_EXECUTABLE}" CACHE FILEPATH
        "Python interpreter for binding cross-validation (needs routing3d_py installed)")
    add_test(NAME bindings
        COMMAND "${ROUTING3D_TEST_PYTHON}"
                "${CMAKE_CURRENT_SOURCE_DIR}/bindings/test_bindings.py"
                "$<TARGET_FILE_DIR:routing3d_cpp>")
endif()
