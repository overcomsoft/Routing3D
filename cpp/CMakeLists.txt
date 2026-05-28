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

add_library(routing3d_core STATIC
    src/occupancy.cpp
    src/cost.cpp
    src/astar.cpp
    src/multi_route.cpp
    src/scene_io.cpp
)
target_include_directories(routing3d_core PUBLIC include)

if(MSVC)
    target_compile_options(routing3d_core PRIVATE /W4 /permissive-)
endif()

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
