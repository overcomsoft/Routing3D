// 씬 입출력 (scene.txt I/O, v1) — Routing3D C++ 엔진 (Phase 3, Step 3.9)
// =============================================================================
// [이 파일이 하는 일]
//   라우팅 씬(입력: 격자/파라미터/장애물/작업)과 결과(출력: 경로/방문/지표)를
//   사람이 읽을 수 있는 텍스트 파일 scene.txt(v1) 로 직렬화/역직렬화한다.
//   규격: docs/spec/scene_format_spec.md, 레퍼런스: routing3d_py/scene_io.py.
//
// [핵심 계약]
//   F2 무손실 왕복: write→read→write 가 **바이트 단위로 동일**. Python↔C++ 교차 동일.
//   F3 \N(None) ↔ ""(빈문자열) 구분 보존(optional 필드).
//   F4 실수는 Python repr(float) 와 **동일 표기**(shortest round-trip)로 기록.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
// =============================================================================
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/route_task.hpp"

namespace routing3d {

// 파일 포맷 식별자/버전 — 헤더에 적고 읽을 때 검증한다.
inline constexpr const char* SCENE_FORMAT_TAG = "routing3d-scene";
inline constexpr int SCENE_FORMAT_VERSION = 1;

// 장애물 1건(AABB + BIM 메타). 점유 레이어 입력. Python obstacle_db.Obstacle 대응.
// 선택 문자열은 \N(None)/""(빈문자열) 구분을 위해 optional.
struct Obstacle {
    Vec3 min_xyz;  // AABB 최소 (mm).
    Vec3 max_xyz;  // AABB 최대 (mm).
    std::optional<std::string> ost_type;
    std::optional<std::string> name;
    std::optional<std::string> object_id;
    std::optional<std::string> ddworks_type;
};

// 작업 1건의 탐색 결과(직렬화 단위). Python AStarResult 의 직렬화 필드 + 경로/방문 레이어.
// path/visited 는 '없음'(미기록)과 '빈 목록'을 구분하기 위해 optional.
struct SceneResult {
    bool success = false;
    double length_mm = 0.0;        // 기하 길이.
    double cost_mm = 0.0;          // 페널티 포함 총비용.
    int turns = 0;
    long long expanded_nodes = 0;
    double elapsed_ms = 0.0;       // 참고용.
    std::optional<std::vector<Cell>> path;     // 경로 레이어.
    std::optional<std::vector<Cell>> visited;  // 방문 레이어(선택).
};

// scene.txt 파일 한 개에 대응하는 메모리 표현. results 는 tasks 와 평행(없으면 nullopt).
struct SceneDoc {
    double cell_mm = 50.0;
    Vec3 origin;
    Cell shape{1, 1, 1};
    RouteParams params;
    std::vector<Obstacle> obstacles;
    // 통과(pass-through) 객체: 점유맵 가시화에는 쓰되 A* 충돌 대상은 아님.
    // occupancy_from_doc / corridor 점유맵에는 넣지 않는다(별도 r3d_copy_passthrough 로만 노출).
    std::vector<Obstacle> passthrough;
    std::vector<RouteTask> tasks;
    std::vector<std::optional<SceneResult>> results;  // tasks 와 같은 길이(또는 비어 있음).
};

// ---- 직렬화 ----
std::string dumps_scene(const SceneDoc& doc);          // SceneDoc → scene.txt 문자열.
void write_scene(const std::string& path, const SceneDoc& doc);  // 파일로 저장(UTF-8/LF).

// ---- 역직렬화 ----
SceneDoc loads_scene(const std::string& text);         // scene.txt 문자열 → SceneDoc.
SceneDoc read_scene(const std::string& path);          // 파일에서 읽기.

// ---- 점유맵 복원 ----
// SceneDoc 의 grid 메타 + obstacles 로 Dense 점유맵을 재구성(점유 레이어 복원).
// 퇴화(두께 0) 박스는 건너뛴다.
DenseOccupancy occupancy_from_doc(const SceneDoc& doc);

// 통과(pass-through) 객체만으로 Dense 점유맵 생성 — 가시화 전용(A* 충돌엔 미사용).
DenseOccupancy occupancy_from_passthrough(const SceneDoc& doc);

// ---- 실수 표기(테스트 노출) ----
// Python repr(float) 와 동일한 최단 왕복 표기를 만든다(F4). 예: 50.0→"50.0", 1e-5→"1e-05".
std::string format_repr_double(double x);

}  // namespace routing3d
