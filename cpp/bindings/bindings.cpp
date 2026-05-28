// pybind11 바인딩 — Routing3D C++ 엔진을 Python 모듈 `routing3d_cpp` 로 노출 (Phase 3, Step 3.10)
// =============================================================================
// [이 파일이 하는 일]
//   C++ 라우팅 엔진(geometry/occupancy/cost/astar/multi_route/scene_io)을 Python 에서
//   직접 호출할 수 있게 pybind11 로 바인딩한다. Python 실험 환경(routing3d_py)과 동일한
//   알고리즘을 C++ 속도로 실행·교차검증하기 위함이다(명세 phase3_plan.md §4.10).
//
// [빌드/실행]  (프로젝트 루트에서; pybind11 은 .venv 에 pip 설치)
//   $env:PYBIND11_DIR = (& .\.venv\Scripts\python.exe -c "import pybind11;print(pybind11.get_cmake_dir())")
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64 `
//         -DBUILD_PYTHON_BINDINGS=ON -Dpybind11_DIR="$env:PYBIND11_DIR" `
//         -DPython_EXECUTABLE="$PWD\.venv\Scripts\python.exe"
//   cmake --build cpp/build --config Release
//   # 생성물: cpp/build/Release/routing3d_cpp*.pyd
//   .\.venv\Scripts\python.exe cpp/bindings/test_bindings.py
//
// [노출 표면]
//   타입 : Cell, Vec3, AABB, RouteParams, DenseOccupancy, AStarResult,
//          RouteTask, PipeResult, MultiRouteResult, Obstacle, SceneResult, SceneDoc
//   함수 : manhattan, count_turns, astar, astar_weighted, order_tasks, route_sequential,
//          dumps_scene, loads_scene, read_scene, write_scene, occupancy_from_doc,
//          format_repr_double
// =============================================================================
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // vector/map/optional ↔ list/dict/None 자동 변환.

#include <map>
#include <optional>
#include <string>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/route_task.hpp"
#include "routing3d/scene_io.hpp"

namespace py = pybind11;
using namespace routing3d;

PYBIND11_MODULE(routing3d_cpp, m) {
    m.doc() = "Routing3D C++ 라우팅 엔진 (Phase 3). 플랜트 배관 3D 직교 라우팅, 단위 mm.";

    // ---------------------------------------------------------------- 기하 타입
    py::class_<Cell>(m, "Cell", "정수 셀 인덱스 (i, j, k).")
        .def(py::init([](int i, int j, int k) { return Cell{i, j, k}; }),
             py::arg("i") = 0, py::arg("j") = 0, py::arg("k") = 0)
        .def_readwrite("i", &Cell::i)
        .def_readwrite("j", &Cell::j)
        .def_readwrite("k", &Cell::k)
        .def("as_tuple", [](const Cell& c) { return py::make_tuple(c.i, c.j, c.k); })
        .def("__eq__", [](const Cell& a, const Cell& b) { return a == b; })
        .def("__repr__", [](const Cell& c) {
            return "Cell(" + std::to_string(c.i) + ", " + std::to_string(c.j) + ", " +
                   std::to_string(c.k) + ")";
        });

    py::class_<Vec3>(m, "Vec3", "월드 좌표/치수 (mm).")
        .def(py::init([](double x, double y, double z) { return Vec3{x, y, z}; }),
             py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("as_tuple", [](const Vec3& v) { return py::make_tuple(v.x, v.y, v.z); })
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + format_repr_double(v.x) + ", " + format_repr_double(v.y) + ", " +
                   format_repr_double(v.z) + ")";
        });

    py::class_<AABB>(m, "AABB", "축 정렬 바운딩 박스(mm). hi > lo (모든 축).")
        .def(py::init<const Vec3&, const Vec3&>(), py::arg("lo"), py::arg("hi"))
        .def_readonly("lo", &AABB::lo)
        .def_readonly("hi", &AABB::hi);

    // ---------------------------------------------------------------- 비용 파라미터
    py::class_<RouteParams>(m, "RouteParams", "라우팅 비용 파라미터(모든 비용 mm, 값 >= 0).")
        .def(py::init([](double cell_mm, double w_turn, double w_clear, int clearance_radius,
                         int clearance_connectivity, std::map<int, double> w_tier) {
                 RouteParams p;
                 p.cell_mm = cell_mm;
                 p.w_turn = w_turn;
                 p.w_clear = w_clear;
                 p.clearance_radius = clearance_radius;
                 p.clearance_connectivity = clearance_connectivity;
                 p.w_tier = std::move(w_tier);
                 return p;
             }),
             py::arg("cell_mm") = 50.0, py::arg("w_turn") = 500.0, py::arg("w_clear") = 10.0,
             py::arg("clearance_radius") = 2, py::arg("clearance_connectivity") = 6,
             py::arg("w_tier") = std::map<int, double>{})
        .def_readwrite("cell_mm", &RouteParams::cell_mm)
        .def_readwrite("w_turn", &RouteParams::w_turn)
        .def_readwrite("w_clear", &RouteParams::w_clear)
        .def_readwrite("clearance_radius", &RouteParams::clearance_radius)
        .def_readwrite("clearance_connectivity", &RouteParams::clearance_connectivity)
        .def_readwrite("w_tier", &RouteParams::w_tier);

    // ---------------------------------------------------------------- 점유맵
    py::class_<DenseOccupancy>(m, "DenseOccupancy", "Dense 점유맵(1B/셀). 단위 mm.")
        .def(py::init<Cell, Vec3, double>(), py::arg("shape"), py::arg("origin"), py::arg("cell_mm"))
        .def("in_bounds", &DenseOccupancy::in_bounds, py::arg("cell"))
        .def("is_blocked", &DenseOccupancy::is_blocked, py::arg("cell"))
        .def("to_world", &DenseOccupancy::to_world, py::arg("cell"))
        .def("to_cell", &DenseOccupancy::to_cell, py::arg("world"))
        .def("block_cell", &DenseOccupancy::block_cell, py::arg("cell"))
        .def("add_box", &DenseOccupancy::add_box, py::arg("box"))
        .def("count_blocked", &DenseOccupancy::count_blocked)
        .def("copy", &DenseOccupancy::copy)
        .def("lin", &DenseOccupancy::lin, py::arg("cell"))
        .def("unlin", &DenseOccupancy::unlin, py::arg("idx"))
        .def_property_readonly("shape", &DenseOccupancy::shape)
        .def_property_readonly("origin", &DenseOccupancy::origin)
        .def_property_readonly("cell_mm", &DenseOccupancy::cell_mm)
        .def_property_readonly("size", &DenseOccupancy::size);

    // ---------------------------------------------------------------- A* 결과
    py::class_<AStarResult>(m, "AStarResult", "A* 탐색 결과(경로/길이/회전/확장수).")
        .def_readonly("success", &AStarResult::success)
        .def_readonly("path", &AStarResult::path)
        .def_readonly("length_mm", &AStarResult::length_mm)
        .def_readonly("turns", &AStarResult::turns)
        .def_readonly("expanded_nodes", &AStarResult::expanded_nodes)
        .def_readonly("cost_mm", &AStarResult::cost_mm)
        .def_readonly("elapsed_ms", &AStarResult::elapsed_ms);

    m.def("manhattan", &manhattan, py::arg("a"), py::arg("b"), "두 셀의 맨해튼 거리(셀 수).");
    m.def("count_turns", &count_turns, py::arg("path"), "경로의 방향 전환 횟수.");
    m.def("astar", &astar, py::arg("occ"), py::arg("start"), py::arg("goal"),
          py::arg("step_cost") = -1.0, py::arg("max_expansions") = -1,
          "균일 비용 직교 A*. step_cost<0 이면 cell_mm 사용.");
    m.def("astar_weighted", &astar_weighted, py::arg("occ"), py::arg("start"), py::arg("goal"),
          py::arg("params"), py::arg("max_expansions") = -1,
          "비용함수 A*(turn penalty/클리어런스/단 분리). 상태=(셀,진입방향).");

    // ---------------------------------------------------------------- 작업/다중 라우팅
    py::class_<RouteTask>(m, "RouteTask", "라우팅 작업 1건(start→end + 유틸리티 메타).")
        .def(py::init([](Vec3 start_mm, Vec3 end_mm, std::optional<std::string> utility,
                         std::optional<std::string> utility_group,
                         std::optional<std::string> start_name, std::optional<std::string> end_name,
                         std::optional<std::string> end_instance_guid) {
                 RouteTask t;
                 t.start_mm = start_mm;
                 t.end_mm = end_mm;
                 t.utility = std::move(utility);
                 t.utility_group = std::move(utility_group);
                 t.start_name = std::move(start_name);
                 t.end_name = std::move(end_name);
                 t.end_instance_guid = std::move(end_instance_guid);
                 return t;
             }),
             py::arg("start_mm"), py::arg("end_mm"), py::arg("utility") = std::nullopt,
             py::arg("utility_group") = std::nullopt, py::arg("start_name") = std::nullopt,
             py::arg("end_name") = std::nullopt, py::arg("end_instance_guid") = std::nullopt)
        .def_readwrite("start_mm", &RouteTask::start_mm)
        .def_readwrite("end_mm", &RouteTask::end_mm)
        .def_readwrite("utility", &RouteTask::utility)
        .def_readwrite("utility_group", &RouteTask::utility_group)
        .def_readwrite("start_name", &RouteTask::start_name)
        .def_readwrite("end_name", &RouteTask::end_name)
        .def_readwrite("end_instance_guid", &RouteTask::end_instance_guid)
        .def("utility_label", &RouteTask::utility_label);

    py::class_<PipeResult>(m, "PipeResult", "배관 1개의 라우팅 결과.")
        .def_readonly("task", &PipeResult::task)
        .def_readonly("result", &PipeResult::result)
        .def_readonly("order_index", &PipeResult::order_index);

    py::class_<MultiRouteResult>(m, "MultiRouteResult", "다중 배관 순차 라우팅 결과 묶음.")
        .def_readonly("pipes", &MultiRouteResult::pipes)
        .def_readonly("occupancy", &MultiRouteResult::occupancy)
        .def_readonly("priority", &MultiRouteResult::priority)
        .def_property_readonly("success_count", &MultiRouteResult::success_count)
        .def_property_readonly("fail_count", &MultiRouteResult::fail_count)
        .def_property_readonly("total_length_mm", &MultiRouteResult::total_length_mm)
        .def_property_readonly("success_rate", &MultiRouteResult::success_rate);

    m.def("order_tasks", &order_tasks, py::arg("occ"), py::arg("tasks"), py::arg("priority"),
          "우선순위 규칙으로 작업 정렬(longest/shortest/utility/original).");
    m.def("route_sequential", &route_sequential, py::arg("occ"), py::arg("tasks"), py::arg("params"),
          py::arg("priority") = "longest", py::arg("pipe_radius") = 0, py::arg("snap_to_free") = 2,
          py::arg("max_expansions") = -1, "배관들을 충돌 없이 순차 라우팅(원본 점유맵 불변).");

    // ---------------------------------------------------------------- scene.txt I/O
    py::class_<Obstacle>(m, "Obstacle", "장애물 1건(AABB + BIM 메타). 선택 문자열은 None 가능.")
        .def(py::init<>())
        .def_readwrite("min_xyz", &Obstacle::min_xyz)
        .def_readwrite("max_xyz", &Obstacle::max_xyz)
        .def_readwrite("ost_type", &Obstacle::ost_type)
        .def_readwrite("name", &Obstacle::name)
        .def_readwrite("object_id", &Obstacle::object_id)
        .def_readwrite("ddworks_type", &Obstacle::ddworks_type);

    py::class_<SceneResult>(m, "SceneResult", "작업별 탐색 결과(직렬화 단위) + 경로/방문 레이어.")
        .def(py::init<>())
        .def_readwrite("success", &SceneResult::success)
        .def_readwrite("length_mm", &SceneResult::length_mm)
        .def_readwrite("cost_mm", &SceneResult::cost_mm)
        .def_readwrite("turns", &SceneResult::turns)
        .def_readwrite("expanded_nodes", &SceneResult::expanded_nodes)
        .def_readwrite("elapsed_ms", &SceneResult::elapsed_ms)
        .def_readwrite("path", &SceneResult::path)
        .def_readwrite("visited", &SceneResult::visited);

    py::class_<SceneDoc>(m, "SceneDoc", "scene.txt 파일 한 개에 대응하는 메모리 표현.")
        .def(py::init<>())
        .def_readwrite("cell_mm", &SceneDoc::cell_mm)
        .def_readwrite("origin", &SceneDoc::origin)
        .def_readwrite("shape", &SceneDoc::shape)
        .def_readwrite("params", &SceneDoc::params)
        .def_readwrite("obstacles", &SceneDoc::obstacles)
        .def_readwrite("tasks", &SceneDoc::tasks)
        .def_readwrite("results", &SceneDoc::results);

    m.def("dumps_scene", &dumps_scene, py::arg("doc"), "SceneDoc → scene.txt 문자열.");
    m.def("loads_scene", &loads_scene, py::arg("text"), "scene.txt 문자열 → SceneDoc.");
    m.def("read_scene", &read_scene, py::arg("path"), "scene.txt 파일 읽기.");
    m.def("write_scene", &write_scene, py::arg("path"), py::arg("doc"), "scene.txt 파일 쓰기(UTF-8/LF).");
    m.def("occupancy_from_doc", &occupancy_from_doc, py::arg("doc"),
          "SceneDoc 의 grid+obstacles 로 Dense 점유맵 복원.");
    m.def("format_repr_double", &format_repr_double, py::arg("x"),
          "Python repr(float) 와 동일한 최단 왕복 표기.");
}
