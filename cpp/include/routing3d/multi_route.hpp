// 다중 배관 순차 라우팅 (Multi-Pipe Sequential Routing) — Routing3D C++ 엔진 (Phase 3, Step 3.5)
// =============================================================================
// [이 파일이 하는 일]
//   여러 배관(start→end 작업)을 '한 개씩 차례로' 라우팅한다. 핵심은 **이미 깔린
//   배관을 다음 배관의 장애물로 추가**(mark_pipe)하여 배관끼리 같은 셀을 점유하지
//   않게(충돌 0) 만드는 것이다. greedy sequential 베이스라인(rip-up/CBS 는 후속 단계).
//   Python 레퍼런스 routing3d_py/multi_route.py 와 1:1 대응, 명세 algorithm_spec.md §5.
//
// [전체 흐름]
//   route_sequential(occ, tasks, params, priority, pipe_radius, snap_to_free):
//     1) order_tasks 로 작업 순서 결정(기본 longest = 맨해튼 거리 긴 것 먼저).
//     2) work = occ.copy()  ← 원본 점유맵 불변(계약 M2).
//     3) 작업마다: start/end 가 점유면 snap → astar_weighted → 성공이면 mark_pipe.
//     4) MultiRouteResult(성공수/실패수/성공률/총길이) 반환.
//   계약 M1(충돌 0): 성공 경로들은 쌍별로 셀을 공유하지 않는다.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/route_task.hpp"

namespace routing3d {

// 배관 1개의 라우팅 결과.
struct PipeResult {
    RouteTask task;        // 라우팅 작업.
    AStarResult result;    // A* 결과(성공 여부/경로/길이/회전/확장수).
    int order_index = 0;   // 실제 라우팅된 순서(0부터).
};

// 다중 배관 순차 라우팅 결과 묶음.
struct MultiRouteResult {
    std::vector<PipeResult> pipes;  // 라우팅 순서대로의 결과.
    DenseOccupancy occupancy;       // 최종 작업용 점유맵(장애물 + 모든 성공 배관).
    std::string priority;           // 사용한 우선순위 규칙 이름.

    int success_count() const;        // 성공한 배관 수.
    int fail_count() const;           // 실패한 배관 수.
    double total_length_mm() const;   // 성공 배관의 기하 길이 합(mm).
    double success_rate() const;      // 성공 비율 (0~1).
};

// ---- 우선순위 ----
// 우선순위 규칙에 따라 작업 순서를 정렬해 반환한다(원본 변경 없음, 안정 정렬).
//   longest  : 맨해튼 거리 긴 것 먼저(기본).  shortest : 짧은 것 먼저.
//   utility  : (유틸 라벨 오름차순, 거리 내림차순).  original : 입력 순서.
std::vector<RouteTask> order_tasks(const DenseOccupancy& occ,
                                   const std::vector<RouteTask>& tasks,
                                   const std::string& priority);

// ---- 순차 라우팅 ----
// 배관들을 순차적으로(충돌 없이) 라우팅한다. occ 는 변경하지 않는다(내부 사본 사용).
//   pipe_radius  : 깔린 배관을 점유로 추가할 때 팽창 반경(셀). 0=경로 셀만.
//   snap_to_free : start/end 가 점유면 빈 셀 탐색 반경(셀).
MultiRouteResult route_sequential(const DenseOccupancy& occ,
                                  const std::vector<RouteTask>& tasks,
                                  const RouteParams& params,
                                  const std::string& priority = "longest",
                                  int pipe_radius = 0,
                                  int snap_to_free = 2,
                                  long long max_expansions = -1);

// ---- 보조 ----
// 경로 셀(+반경 radius 6-이웃)을 점유로 표시한다(다음 배관이 피하도록).
void mark_pipe(DenseOccupancy& occ, const std::vector<Cell>& path, int radius);

// cell 이 점유면 반경 radius 내 가장 가까운 빈 셀 반환(없으면 원래 cell).
// 거리 동률은 (di,dj,dk) 사전순 첫 셀(Python 과 동일 결정성).
Cell snap_to_free_cell(const DenseOccupancy& occ, Cell cell, int radius);

}  // namespace routing3d
