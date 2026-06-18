// 씬 직렬화/역직렬화 (scene.txt I/O, v1) — Routing3D C++ 엔진 (Phase 3, Step 3.9)
// =============================================================================
// [이 파일이 하는 일]
//   라우팅 입력: 격자/파라미터/장애물/작업)과 결과(출력: 경로/방문/지표)를
//   사람이 읽을 수 있는 텍스트 파일 scene.txt(v1) 로 직렬화·역직렬화한다.
//   규격: docs/spec/scene_format_spec.md, 레퍼런스: routing3d_py/scene_io.py.
//
// [핵심 계약]
//   F2 무손실 왕복: write→read→write 가 **바이트 단위로 동일**. Python과C++ 교차 동일.
//   F3 \N(None) 과 ""(빈문자열) 구분 보존(optional 필드).
//   F4 부동소수점은 Python repr(float) 과 **동일 표기**(shortest round-trip)로 기록.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
//
// =============================================================================
// [전체 모듈 개요]
//
// scene_io 는 Routing3D 엔진의 **영속화(Persistence)** 계층이다.
// 엔진의 핵심 데이터 모델인 SceneDoc 을 텍스트로 직렬화·역직렬화하여
// Python ↔ C++ ↔ C# 세 구현이 동일한 씬 파일을 공유하도록 한다.
//
// [SceneDoc — 중앙 데이터 모델]
//   SceneDoc 은 한 씬(scene) 의 **모든 정보**를 담는 구조체다:
//     - 격자 메타(cell_mm, origin, shape)
//     - 라우팅 파라미터(RouteParams)
//     - 장애물 목록(Obstacle) + 통과 허용(passthrough)
//     - 라우팅 작업(RouteTask) + 결과(SceneResult)
//   엔진의 모든 데이터 흐름은 SceneDoc 을 경유한다:
//     DB/CLI → SceneDoc → occupancy_from_doc → 점유맵 → A* → SceneResult → SceneDoc → 파일
//
// [포맷 버전 관리]
//   SCENE_FORMAT_TAG / SCENE_FORMAT_VERSION 이 헤더에 기록되어
//   로더가 버전 불일치를 조기 감지한다.
//
// [인코딩 규칙]
//   파일은 UTF-8, 줄 끝 LF(\n). 부동소수는 format_repr_double() 로 기록.
//   문자열 None ↔ \N, 빈문자열 ↔ "" (F3 계약).
//
// [의존 관계]
//   cost.hpp (RouteParams), geometry.hpp (Vec3/Cell/AABB),
//   occupancy.hpp (DenseOccupancy), route_task.hpp (RouteTask)
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

// =============================================================================
// 포맷 식별자 상수
// =============================================================================
// scene.txt 파일의 첫 줄에 태그+버전을 기록하여 로더가 파일 종류를 검증한다.
// 버전이 맞지 않으면 로더가 예외를 던지므로 구버전 파일 혼용을 조기 감지한다.

// 파일 포맷 식별 태그. 파싱 전 파일 종류 확인용.
inline constexpr const char* SCENE_FORMAT_TAG = "routing3d-scene";
// 현재 지원하는 포맷 버전. 헤더에서 읽어 불일치 시 예외 발생.
inline constexpr int SCENE_FORMAT_VERSION = 3;

// =============================================================================
// Obstacle — 장애물 1건 (AABB + BIM 메타)
// =============================================================================
// 배관 라우팅을 방해하는 공간 장애물을 표현한다.
// PostgreSQL TB_BIM_OBSTACLE 의 한 행이 이 구조체 한 개에 대응한다.
//
// 불변식:
//   min_xyz < max_xyz (모든 축) — AABB 와 동일 조건.
//   크기가 0인(두께 없는) 박스는 점유맵 add_box 에서 자동으로 건너뛴다.
//
// 선택 문자열 필드:
//   std::optional<std::string> 를 사용해 Python None(\N) 과 빈문자열("") 을
//   구분한다(F3 계약). nullopt = \N, some("") = "", some("value") = 값.
//
// 사용처:
//   SceneDoc.obstacles → occupancy_from_doc → DenseOccupancy.add_box
//   SceneDoc.passthrough → occupancy_from_passthrough (A* 충돌에 미사용)
struct Obstacle {
    // 장애물 AABB 의 최소 코너 (mm). TB_BIM_OBSTACLE.AABB_MIN_*.
    Vec3 min_xyz;
    // 장애물 AABB 의 최대 코너 (mm). TB_BIM_OBSTACLE.AABB_MAX_*.
    Vec3 max_xyz;
    // DDWorks 객체 타입 문자열 (예: "Wall", "Floor"). 라우팅에 미사용, 로깅용.
    std::optional<std::string> ost_type;
    // 장애물 이름 (예: "Damper-FMPVC-150A"). 디버그·UI 표시용.
    std::optional<std::string> name;
    // BIM 객체 고유 ID. 장애물과 PoC 인스턴스를 연결하는 키.
    std::optional<std::string> object_id;
    // DDWorks 상세 타입 (예: "SLAB", "BEAM"). 복셀화 우선순위 판단용.
    std::optional<std::string> ddworks_type;
};

// =============================================================================
// SceneResult — 작업 1건의 탐색 결과 (직렬화 단위)
// =============================================================================
// A* 가 한 RouteTask 를 라우팅한 결과를 담는다.
// Python AStarResult 의 직렬화 필드 + 경로/방문 데이터를 포함한다.
//
// path/visited 는 '없음'(미기록)과 '빈 목록'을 구분하기 위해 optional 로 선언:
//   - nullopt: 결과 없음 (탐색 미실행 또는 직렬화 미포함)
//   - some([]): 탐색했으나 경로가 빈 경우 (이론상 드뭄)
//   - some([Cell,...]): 정상 경로 또는 방문 셀 목록
//
// fail 필드:
//   RouteFail enum 값(A1). 성공=0. scene.txt 미직렬화(F2 불변식 밖),
//   C# 뷰어가 ExplainFailure 표시에 사용.
struct SceneResult {
    // 탐색 성공 여부. false 이면 length_mm/turns 은 무의미.
    bool success = false;
    // 경로의 기하 길이 (mm). 셀 중심 간 맨해튼 거리 × cell_mm 의 합.
    double length_mm = 0.0;
    // 페널티를 포함한 총 비용 (mm 환산). A* g 값의 최종값.
    double cost_mm = 0.0;
    // 방향 전환(꺾임) 횟수. 직선 구간 수 - 1.
    int turns = 0;
    // A* 가 확장(팝)한 노드 수. 탐색 난이도 지표.
    long long expanded_nodes = 0;
    // 탐색 소요 시간 (ms). 벤치마크 참고용 — 재현 보장 없음.
    double elapsed_ms = 0.0;
    // 실패 사유 (RouteFail enum 값, A1). 성공=0. scene.txt 미직렬화(F2 불변식 외).
    int fail = 0;
    // 경로 셀 목록. nullopt=미기록, some([])=빈 경로, some([...])=정상 경로.
    std::optional<std::vector<Cell>> path;
    // 방문 셀 목록 (옵션). collect_visited=true 시에만 채워짐(A2: 기본 OFF).
    std::optional<std::vector<Cell>> visited;
};

// =============================================================================
// SceneDoc — 씬 파일 전체의 메모리 표현 (중앙 데이터 모델)
// =============================================================================
// scene.txt 파일 1개에 대응하는 모든 데이터를 담는 구조체.
// results 는 tasks 와 길이가 같거나(1:1 대응) 비어 있다(미라우팅).
//
// 데이터 흐름:
//   [입력] DB / CLI 인자 → SceneDoc 구성
//     → occupancy_from_doc() → 점유맵
//     → A* 라우팅 → SceneResult 생성
//     → SceneDoc.results 채움
//   [출력] write_scene() → scene.txt (바이트 단위 재현 가능)
//
// 설계 원칙:
//   값 타입(모든 필드가 값 소유). 복사 가능하지만 크기가 크므로
//   함수 경계에서는 const 참조로 전달한다.
struct SceneDoc {
    // 셀 크기 (mm). 기본 50mm. 작을수록 정밀하나 격자가 커진다.
    // 실사용: 100mm(빠름), 50mm(기본), 25mm(정밀), 10mm(초정밀·ImplicitOccupancy 필요).
    double cell_mm = 50.0;
    // 격자 원점 — 셀 (0,0,0) 의 최소 코너 좌표 (mm).
    // DB 씬 로드 시 그룹 AABB lo 코너로 설정(3축 클램프, §8 참고).
    Vec3 origin;
    // 격자 형상 — 각 축의 셀 수. 전체 셀 수 = shape.i * shape.j * shape.k.
    // 5M 초과 시 ImplicitOccupancy 자동 전환(capi 로직).
    Cell shape{1, 1, 1};
    // 라우팅 파라미터 (가중치, 클리어런스, 방향 페널티 등). cost.hpp 참조.
    RouteParams params;
    // 장애물 목록. 점유맵에 복셀화되어 A* 가 회피한다.
    std::vector<Obstacle> obstacles;
    // 통과(pass-through) 객체: 점유맵에 가시화되나 A* 충돌 대상이 아님.
    // occupancy_from_doc / corridor 점유맵에서는 무시됨.
    // (별도 r3d_copy_passthrough 로만 노출 — COLLISION_PASS=1 장애물용).
    std::vector<Obstacle> passthrough;
    // 라우팅 작업 목록. 각 RouteTask = 출발 PoC → 종단 PoC 배관 1건.
    std::vector<RouteTask> tasks;
    // 결과 목록. tasks 와 같은 길이이거나 비어 있음(미라우팅 시 nullopt 요소).
    std::vector<std::optional<SceneResult>> results;
};

// =============================================================================
// 직렬화 함수
// =============================================================================

// -------------------------------------------------------------------
// dumps_scene — SceneDoc → JSON 문자열 직렬화
// -------------------------------------------------------------------
// SceneDoc 전체를 scene.txt v1 포맷의 텍스트 문자열로 변환한다.
// F2: 동일 SceneDoc 을 dumps → loads → dumps 하면 문자열이 동일해야 한다.
// F4: 부동소수는 format_repr_double() 로 Python repr(float) 과 동일하게 기록.
//
// 시간복잡도: O(장애물 수 + 작업 수 + 경로 셀 수 합계)
std::string dumps_scene(const SceneDoc& doc);

// -------------------------------------------------------------------
// write_scene — SceneDoc → 파일 기록 (UTF-8/LF)
// -------------------------------------------------------------------
// path 경로에 scene.txt 를 UTF-8, 줄 끝 LF 로 기록한다.
// 내부적으로 dumps_scene() 을 호출한다.
//
// 예외: 파일 열기 실패 시 std::runtime_error 발생.
// 시간복잡도: O(직렬화 크기)
void write_scene(const std::string& path, const SceneDoc& doc);

// =============================================================================
// 역직렬화 함수
// =============================================================================

// -------------------------------------------------------------------
// loads_scene — JSON 문자열 → SceneDoc 역직렬화
// -------------------------------------------------------------------
// scene.txt v1 포맷의 텍스트 문자열을 SceneDoc 으로 파싱한다.
// legacy scene.txt 입력(구버전)도 호환 읽기가 가능하다.
//
// F3: \N → nullopt, "" → some(""), 값 → some(값) 으로 복원.
// 예외: 포맷 불일치·버전 오류 시 std::runtime_error 발생.
// 시간복잡도: O(입력 텍스트 크기)
SceneDoc loads_scene(const std::string& text);

// -------------------------------------------------------------------
// read_scene — 파일 → SceneDoc 역직렬화
// -------------------------------------------------------------------
// path 파일을 UTF-8 로 읽어 loads_scene() 에 전달한다.
//
// 예외: 파일 열기 실패 또는 파싱 오류 시 std::runtime_error 발생.
// 시간복잡도: O(파일 크기)
SceneDoc read_scene(const std::string& path);

// =============================================================================
// 점유맵 복원 함수
// =============================================================================

// -------------------------------------------------------------------
// occupancy_from_doc — SceneDoc → DenseOccupancy 재구성
// -------------------------------------------------------------------
// SceneDoc 의 격자 메타(cell_mm, origin, shape) + obstacles 목록으로
// DenseOccupancy 를 새로 구성한다.
// 두께가 0인(크기 0) 박스는 건너뛴다.
//
// 사용처:
//   - A* 라우팅 전 초기 점유맵 생성 (장애물 복셀화)
//   - 테스트 픽스처에서 scene.txt → 점유맵 복원
//
// 반환: 새로 할당된 DenseOccupancy (shape 크기의 배열, 1B/셀)
// 시간복잡도: O(장애물 수 × 장애물 당 평균 복셀 수)
DenseOccupancy occupancy_from_doc(const SceneDoc& doc);

// -------------------------------------------------------------------
// occupancy_from_passthrough — 통과 객체만으로 DenseOccupancy 생성
// -------------------------------------------------------------------
// SceneDoc.passthrough 의 장애물만으로 점유맵을 구성한다.
// A* 충돌에는 미사용 — 가시화·참조 전용(COLLISION_PASS=1 장애물).
//
// 반환: passthrough 장애물만 복셀화된 DenseOccupancy
// 시간복잡도: O(passthrough 수 × 장애물 당 평균 복셀 수)
DenseOccupancy occupancy_from_passthrough(const SceneDoc& doc);

// =============================================================================
// 부동소수 표기 유틸리티
// =============================================================================

// -------------------------------------------------------------------
// format_repr_double — Python repr(float) 과 동일한 최단 왕복 표기 생성
// -------------------------------------------------------------------
// Python 의 repr(float) 과 동일하게 최단 round-trip 가능 문자열을 만든다(F4).
// 예: 50.0 → "50.0",  1e-5 → "1e-05",  -0.0 → "-0.0"
//
// 이 함수가 필요한 이유:
//   C++ 의 기본 printf("%g") 나 std::to_string() 은 Python repr 과 다른
//   표기(예: 자릿수 차이)를 생성하여 바이트 단위 무손실 왕복(F2)을 깨뜨린다.
//   C++17 std::to_chars(chars_format::general, precision=최소) 로 구현하여
//   Python repr 과 동일한 최단 표기를 보장한다.
//
// 시간복잡도: O(1)
std::string format_repr_double(double x);

}  // namespace routing3d