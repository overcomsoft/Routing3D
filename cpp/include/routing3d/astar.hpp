// 직교 A* 탐색 — Routing3D C++ 엔진 (Phase 3, Step 3.3/3.4/3.6)
// =============================================================================
// [이 파일이 하는 일]
//   점유맵 위에서 6방향 직교 최단 경로를 찾는다.
//   - astar          : 균일 비용(상태=셀).            명세 algorithm_spec.md §3
//   - astar_weighted : 비용함수(상태=(셀,진입방향)).  명세 algorithm_spec.md §4
//   휴리스틱 = manhattan × cell_mm (admissible & consistent). 단위 mm.
//   결정성(A2/W1): (f, 삽입순서 counter) tie-break + 고정 이웃 순서 → 재현 가능.
//
//   Step 3.6: 점유맵 백엔드(Dense/Sparse/Vdb)에 **무관**하도록 템플릿화(헤더 정의).
//   백엔드는 in_bounds/is_blocked/lin/unlin/size/cell_mm 만 제공하면 된다.
//   주의: g/closed 를 lin() 선형 인덱스 배열로 잡으므로 '경계가 한정된'(작은/corridor) 격자용.
//         초대형 격자 전역 탐색은 후속(계층 corridor)에서 해시 기반으로 확장한다.
//
// [핵심 설계 결정 목록]
//
//   1) 상태 공간 확장 — 방향 추가(Step 3.4):
//      균일 astar 의 상태 = {셀}. 비용함수 astar_weighted 의 상태 = {셀, 진입방향}.
//      진입방향을 상태에 포함해야 꺾임 페널티(w_turn)를 정확하게 계산할 수 있다.
//      dir ∈ {-1(시작), 0..5(NEIGHBORS_6 인덱스)} → 7가지 → 상태키 = lin*7 + (dir+1).
//
//   2) 코너 최소직선 — run 추가(Step P3u):
//      min_straight_cells > 1 이면 상태에 run(현재 방향으로 연속 직진한 셀 수)을 추가.
//      상태키 = (lin*7 + (dir+1)) * RS + run (RS = min_run+1).
//      run < min_run 이면 꺾임(방향 전환) 금지 → 엘보 간 최소 단관 보장(하드 제약).
//      비활성(RS=1, run=0): 기존 상태키와 수치적으로 동일 → 골든 바이트 완전 불변.
//
//   3) 결정성 보장(A2/W1):
//      PQ에 (f, counter) 쌍으로 tie-break → f가 같을 때 먼저 삽입된 상태가 먼저 처리.
//      NEIGHBORS_6은 고정 순서(+x,-x,+y,-y,+z,-z) → 동일 입력 → 동일 확장 순서 → 동일 경로.
//
//   4) 평면 해시맵(Tier-3 Stage 3a):
//      기존 g/came/closed 세 개의 노드기반 std::unordered_map/set을 StateMap 하나로 통합.
//      연속 배열 + 선형탐사 → 메모리 ~3× 감소, 캐시 친화. 결정성은 PQ가 보장하므로 결과 불변.
//
//   5) 동적 가중(수렴) A*:
//      정적 w_heur 대신, 목표 근처에서 가중을 w_heur_near로 낮춰 그리디 함정 완화.
//      먼 곳=빠름(w_heur), 목표 근처=정확(w_heur_near). 무제한 탐색에서만 활성.
//
// [실행 명령어]
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release  # golden/capi/implicit 등 13/13
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"

namespace routing3d {

// RouteFail — 라우팅 실패 사유 열거형(Step P3q, A1)
//
// success=false일 때 '왜' 실패했는지 분류한다.
// 디버깅·제품 UI(C# ExplainFailure 패널)·진단 리포트에 사용한다.
// 값은 C ABI(R3dResult.fail_reason int)와 C#(RouteResult.Fail)으로 그대로 전달된다.
// 성공이면 None(0). 실패 사유는 원인 우선순위 순으로 검사한다(아래 순서로).
enum class RouteFail : int {
    // 성공(또는 미시도). success=true이면 항상 None.
    None = 0,
    // 시작 셀이 장애물이거나 격자 밖. PoC가 장애물에 묻혔을 때 발생.
    // 수정: snap_to_free_cell로 PoC를 최근접 자유 셀로 스냅(P3j 전처리).
    StartBlocked = 1,
    // 목표 셀이 장애물이거나 격자 밖. 종단(덕트/레터럴)이 묻혔을 때 발생.
    GoalBlocked = 2,
    // 시작 또는 목표가 하드 corridor 튜브(계층 라우팅의 fine 탐색 영역) 밖.
    // 호출자가 튜브 반경을 ×2로 키워 재시도(Stage 2 robust 재시도).
    CorridorMiss = 3,
    // 탐색 확장 상한(max_expansions) 도달. 거대 격자에서 어려운 배관이 상한을 넘음.
    // env R3D_MAX_EXP로 상한 상향 가능. 계층 corridor escalation으로 우회 가능.
    ExpansionLimit = 4,
    // 목표 셀에 닿았으나 요구된 진입축(goal_dir)으로 진입하지 못함.
    // 장애물이 해당 방향 진입을 막은 경우. 호출자가 goal_dir=-1(무제약)로 재시도.
    GoalDirBlocked = 5,
    // 탐색 고갈(열린 집합이 비어 있음) = 연결 경로 없음. 국소 차단 또는 혼잡.
    // rip-up & reroute(route_multi_impl 인라인 rip-up, P3r CBS) 대상.
    NoPath = 6,
};

// AStarResult — A* 탐색 결과 구조체
//
// astar 및 astar_weighted 함수가 반환하는 값의 컨테이너.
// 성공(success=true)이면 path가 채워지고, 실패(success=false)이면 path는 비어 있다.
// C ABI(R3dResult)와 C#(RouteResult)으로 각 필드가 개별 전달된다.
struct AStarResult {
    // 탐색 성공 여부. true이면 path[0]=start, path.back()=goal이 보장된다.
    bool success = false;

    // 최적(또는 준최적) 경로 셀 목록. [start, ..., goal] 순서(start/goal 포함).
    // 실패 시 비어 있음. 인접 셀들은 정확히 1셀 거리(6방향 이웃).
    std::vector<Cell> path;

    // 경로의 실제 물리 길이(mm) = (path.size() - 1) × cell_mm. 페널티 미포함.
    // 0셀(start==goal) 이면 0.0.
    double length_mm = 0.0;

    // 경로의 방향 전환(꺾임) 횟수. count_turns(path)와 동일.
    // path가 비어 있거나 직선이면 0.
    int turns = 0;

    // 탐색 중 확장(closed에 추가)한 상태 수. 탐색 복잡도 지표.
    // 작을수록 빠른 탐색. 상한(max_expansions) 도달 시 이 값이 상한에 가깝다.
    long long expanded_nodes = 0;

    // 페널티 포함 총 비용(mm) = g(goal). 균일 비용이면 length_mm과 동일.
    // 비용함수(꺾임·클리어런스·회랑) 적용 시에는 length_mm보다 크다.
    double cost_mm = 0.0;

    // 탐색 소요 시간(밀리초). 벤치마크/진단용.
    double elapsed_ms = 0.0;

    // 실패 사유(A1). success=false일 때만 의미 있음. 성공=None.
    // C ABI R3dResult.fail_reason으로 그대로 전달. C# ExplainFailure 패널에 표시.
    RouteFail fail = RouteFail::None;

    // 방문(확장)한 셀 목록 — collect_visited=true일 때만 채워진다(셀 단위 중복 제거).
    // 동일 셀이 여러 진입방향으로 expand되어도 visited에는 한 번만 기록(visited_seen 참조).
    // 용도: 뷰어 '방문맵 레이어' 시각화, scene.txt [visited] 섹션 출력.
    // 기본 false(A2 opt-in): 전체 배치 라우팅에서 collect_visited 끔으로 오버헤드 0.
    // 뷰어가 단계탐색 애니메이션 요청 시에만 SetCollectVisited(true) 호출.
    std::vector<Cell> visited;
};

// AllowAll — 모든 셀을 허용하는 corridor 술어(astar_weighted의 기본값)
//
// 계층 corridor 라우팅(route_hier)에서는 fine A*가 coarse 골격 ±r 튜브 내 셀만
// 탐색하도록 InCorridor 함수 객체를 주입한다(하드 제한 — 튜브 밖은 즉시 skip).
//
// 기본값인 AllowAll은 항상 true를 반환하므로:
//   - `if (!in_corridor(nb)) continue;` 분기가 항상 false → 컴파일러가 dead branch로 제거.
//   - 기존 전역 탐색 호출은 코드 변경 없이 그대로 동작(결과·골든 완전 불변).
struct AllowAll {
    bool operator()(const Cell&) const noexcept { return true; }
};

using AStarTraceFn = std::function<void(const char* event, const Cell& from, const Cell& to,
                                        long long expanded, int dir, int run, int required)>;

// count_turns — 경로의 방향 전환(꺾임) 횟수를 센다. 백엔드 무관, 헤더 인라인.
// path의 연속 세 점(p[i-1], p[i], p[i+1])에서 방향 오프셋이 바뀌면 꺾임 1회.
// path.size() < 3이면 꺾임 없음(직선 또는 제자리). O(n) 시간.
inline int count_turns(const std::vector<Cell>& path) {
    if (path.size() < 3) return 0;
    int turns = 0;
    Cell prev{path[1].i - path[0].i, path[1].j - path[0].j, path[1].k - path[0].k};
    for (size_t n = 2; n < path.size(); ++n) {
        Cell cur{path[n].i - path[n - 1].i, path[n].j - path[n - 1].j, path[n].k - path[n - 1].k};
        if (!(cur == prev)) ++turns;
        prev = cur;
    }
    return turns;
}

namespace detail {

using Clock = std::chrono::steady_clock;

// ms_since — 시작 시각 t0 부터 현재까지 경과한 시간(밀리초). 탐색 소요 시간 측정용.
inline double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// PQItem — 우선순위 큐(open set)에 저장되는 A* 탐색 항목.
//
// [필드]
//   f       : A* 평가함수 f = g + h. 최소 힙 정렬 기준.
//   counter : 삽입 순서(단조 증가). f가 같을 때 먼저 삽입된 항목이 우선 처리(결정성 A2).
//   cell    : 이 항목이 나타내는 격자 셀.
//   dir     : 이 셀에 진입한 방향(NEIGHBORS_6 인덱스, -1=시작).
//   run     : 현재 방향(dir)으로 연속 직진한 셀 수. min_straight_cells>1일 때만 의미.
//             비활성(enforce_run=false)이면 항상 0(상태키에 포함되지 않음).
//
// [결정성(A2/W1)]
//   동일 f를 가진 항목이 여럿일 때 counter가 작은(먼저 삽입된) 항목을 처리.
//   이는 BFS와 유사한 FIFO 순서를 보장해, 동일 입력에 항상 동일한 경로를 생성한다.
//   counter가 없으면 PQ 내부 구현에 따라 f가 같은 항목의 처리 순서가 비결정적이 된다.
struct PQItem {
    double f;
    long long counter;
    Cell cell;
    int dir;
    int run = 0;
};

// PQCmp — PQItem 비교 함수(최소 힙용).
// std::priority_queue는 기본이 최대 힙이므로, 작은 f를 top으로 만들기 위해 부등호를 반전시킨다.
// f가 같을 때: counter가 작은(먼저 삽입된) 항목이 top(결정성 보장).
struct PQCmp {
    bool operator()(const PQItem& a, const PQItem& b) const {
        if (a.f != b.f) return a.f > b.f;  // 작은 f 가 top
        return a.counter > b.counter;       // 동률이면 먼저 삽입된 것이 top
    }
};

// StateMap — 오픈어드레싱 평면 해시맵(Tier-3 Stage 3a)
//
// [배경 — 왜 만들었는가]
//   기존 astar_weighted는 탐색 상태 관리에 세 개의 노드기반 컨테이너를 썼다:
//     std::unordered_map<long long, double>  g       — 상태 → g값
//     std::unordered_map<long long, long long> came  — 상태 → 부모 상태
//     std::unordered_set<long long>          closed  — 확정 상태 집합
//   노드기반 해시맵은 엔트리당 힙 노드(키+값+next 포인터+할당 오버헤드)로 메모리가 ~3×이고,
//   포인터 추적으로 캐시미스가 잦아 10mm/25mm 거대격자에서 메모리·속도 병목이 됐다.
//
// [해법 — 오픈어드레싱 평면 해시맵]
//   하나의 연속 배열(Slot 벡터)에 g/parent/closed를 한 슬롯으로 합쳐 관리.
//   충돌 해소: 선형탐사(linear probing). 부하율(load factor) 0.7 초과 시 2배 grow.
//   해시 함수: splitmix64 finalizer — 정수 키의 하위 비트 편중을 완화.
//   → 메모리 ~3× 감소, 캐시 친화(연속 배열이라 prefetch 효율적).
//   → 실측(Stage1+2+3a, cell=25): 199s → 86.6s (2.3× 가속).
//
// [결정성]
//   결정성은 PQ(f, counter) tie-break이 보장한다.
//   해시맵 구현을 교체해도 PQ가 동일한 순서로 상태를 처리하므로 결과/expanded_nodes 불변(골든 불변).
//
// [주의사항]
//   emplace가 grow(재할당)을 유발할 수 있으므로,
//   반환된 Slot& 또는 Slot*를 emplace 호출 너머(이후)까지 보관하면 dangling이 된다.
//   astar_weighted에서는 `sl.g = t` 등을 emplace 직후에 처리하고, 이후 새 emplace가 있으면
//   먼저 필요한 값을 값 복사(g_st)해 참조를 보존하지 않는다.
//
// [Slot 필드]
//   key    : 상태 키(state_of 계산값). EMPTY(long long::min)이면 빈 슬롯.
//   g      : 이 상태까지의 비용 g.
//   parent : 부모 상태 키(-1이면 시작 상태, 경로 역추적에 사용).
//   closed : true이면 확정(이미 최적 g로 처리됨) — 재처리 skip.
struct StateMap {
    struct Slot { long long key; double g; long long parent; bool closed; };
    std::vector<Slot> s;
    size_t mask = 0, cnt = 0;
    // EMPTY 센티넬: long long의 최솟값. 정상 상태 키는 (lin*7+...)×RS+run ≥ 0이므로 충돌 없음.
    static constexpr long long EMPTY = (std::numeric_limits<long long>::min)();
    StateMap() { reset(256); }
    // reset — 배열을 cap 이상의 2의 거듭제곱 크기로 초기화. 기존 데이터 폐기.
    void reset(size_t cap) {
        size_t n = 16;
        while (n < cap) n <<= 1;
        s.assign(n, Slot{EMPTY, 0.0, -1, false});
        mask = n - 1; cnt = 0;
    }
    // mix — splitmix64 finalizer: 정수 키를 균등하게 분산시켜 클러스터링 억제.
    // 선형탐사는 클러스터링에 민감하므로 해시 품질이 중요하다.
    static inline size_t mix(long long k) {
        uint64_t x = static_cast<uint64_t>(k);
        x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
        x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL; x ^= x >> 33;
        return static_cast<size_t>(x);
    }
    // find — 키 k를 가진 슬롯 포인터 반환. 없으면 nullptr.
    // 선형탐사: mix(k) & mask에서 시작해 EMPTY 슬롯을 만날 때까지 +1씩 전진.
    Slot* find(long long k) {
        size_t i = mix(k) & mask;
        while (s[i].key != EMPTY) { if (s[i].key == k) return &s[i]; i = (i + 1) & mask; }
        return nullptr;
    }
    // emplace — 키 k의 슬롯을 찾거나 새로 삽입. inserted=true이면 새 슬롯(g=0, parent=-1, closed=false).
    // 부하율이 0.7을 초과하면 grow()를 먼저 호출(재할당 → 이전 참조 무효화 주의).
    Slot& emplace(long long k, bool& inserted) {
        if ((cnt + 1) * 10 >= s.size() * 7) grow();   // load factor 0.7 초과 시 2배 grow.
        size_t i = mix(k) & mask;
        while (s[i].key != EMPTY) {
            if (s[i].key == k) { inserted = false; return s[i]; }
            i = (i + 1) & mask;
        }
        s[i] = Slot{k, 0.0, -1, false};
        ++cnt; inserted = true; return s[i];
    }
    // grow — 배열 크기를 2배로 늘리고 기존 항목을 재해시(rehash). O(n) 시간.
    // grow 후 이전에 얻은 Slot& / Slot*는 무효(dangling)가 된다.
    void grow() {
        std::vector<Slot> old = std::move(s);
        s.assign(old.size() * 2, Slot{EMPTY, 0.0, -1, false});
        mask = s.size() - 1; cnt = 0;
        for (const Slot& o : old)
            if (o.key != EMPTY) {
                size_t i = mix(o.key) & mask;
                while (s[i].key != EMPTY) i = (i + 1) & mask;
                s[i] = o; ++cnt;
            }
    }
};

}  // namespace detail

// ---------------------------------------------------------------- 균일 비용 A*
//
// 모든 이동 비용이 동일한 단순 A*(상태=셀). 기본 테스트·골든 검증용.
// 꺾임 페널티·클리어런스·회랑 등 비용함수 없이 순수 맨해튼 최단 경로만 찾는다.
// 결정성: (f, counter) tie-break + NEIGHBORS_6 고정 순서.
//
// [매개변수]
//   step_cost      : 1셀 이동 비용(mm). < 0이면 occ.cell_mm() 사용.
//   max_expansions : 확장 상한. < 0이면 무제한.
//   collect_visited: 방문 셀 수집 여부(시각화용).
template <class Occ>
AStarResult astar(const Occ& occ, Cell start, Cell goal, double step_cost = -1.0,
                  long long max_expansions = -1, bool collect_visited = false) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double sc = (step_cost < 0.0) ? occ.cell_mm() : step_cost;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    auto h = [&](const Cell& c) { return manhattan(c, goal) * sc; };

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    std::unordered_map<int, double> g;
    std::unordered_map<int, Cell> came;  // nb 의 lin → 직전 cell
    std::vector<uint8_t> closed(static_cast<size_t>(occ.size()), 0);

    long long counter = 0;
    g[occ.lin(start)] = 0.0;
    open.push({h(start), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        int cl = occ.lin(cur.cell);
        if (closed[static_cast<size_t>(cl)]) continue;
        closed[static_cast<size_t>(cl)] = 1;
        ++expanded;
        if (collect_visited) R.visited.push_back(cur.cell);

        if (cur.cell == goal) {
            std::vector<Cell> path{goal};
            Cell c = goal;
            auto it = came.find(occ.lin(c));
            while (it != came.end()) {
                c = it->second;
                path.push_back(c);
                it = came.find(occ.lin(c));
            }
            for (size_t a = 0, b = path.size() - 1; a < b; ++a, --b) std::swap(path[a], path[b]);
            R.success = true;
            R.path = std::move(path);
            R.length_mm = (R.path.size() - 1) * sc;
            R.cost_mm = R.length_mm;
            R.turns = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms = detail::ms_since(t0);
            return R;
        }
        if (max_expansions > 0 && expanded >= max_expansions) break;

        double g_cur = g[cl];
        for (const Cell& d : NEIGHBORS_6) {
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};
            if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;
            int nl = occ.lin(nb);
            if (closed[static_cast<size_t>(nl)]) continue;
            double t = g_cur + sc;
            auto git = g.find(nl);
            if (git == g.end() || t < git->second) {
                g[nl] = t;
                came[nl] = cur.cell;
                open.push({t + h(nb), counter++, nb, -1});
            }
        }
    }
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    return R;
}

// ------------------------------------------------------------- 비용함수 A* (핵심 알고리즘)
//
// [함수 개요]
//   RouteParams 비용함수(꺾임·클리어런스·회랑·단 분리·코너 최소직선)를 적용한
//   weighted A* 탐색. Routing3D 엔진의 핵심 경로탐색 함수.
//
// [상태 공간]
//   기본 상태 = (셀, 진입방향 dir). dir ∈ {-1=시작, 0..5=NEIGHBORS_6 인덱스}.
//   상태키 = lin * 7 + (dir+1).   (lin: 셀의 선형 인덱스, long long으로 오버플로 방지)
//   → 상태 수 = 셀 수 × 7. 방향을 포함해야 꺾임 페널티가 정확하게 계산된다.
//
//   코너 최소직선(min_straight_cells > 1) 활성 시 상태에 run 추가:
//   상태키 = (lin * 7 + (dir+1)) * RS + run.   (RS = min_run+1, run ∈ [0, min_run])
//   → 상태 수 = 셀 수 × 7 × RS. 탐색 시간 ×RS 증가.
//   비활성(RS=1, run=0) 시 키 계산이 기존과 수치적으로 동일 → 골든 바이트 완전 불변.
//
// [admissibility 및 최적성]
//   w_heur=1.0: 휴리스틱 h = manhattan × cell_mm ≤ 실제 비용(admissible) → 최적해 보장.
//   w_heur>1.0: 준최적(최적의 w_heur 배 이내). 탐색량 급감(실측 10×+).
//
// [매개변수]
//   occ            : 점유맵 백엔드(in_bounds/is_blocked/lin/unlin/cell_mm 제공).
//   start/goal     : 시작·목표 셀.
//   params         : 비용 파라미터(RouteParams — cell_mm, w_turn, w_clear, w_heur 등).
//   max_expansions : 확장 상한. ≤ 0이면 무제한. 거대격자에서는 양수로 제한(메모리 보호).
//   collect_visited: 방문 셀 수집(시각화용). 기본 false(A2 opt-in, 오버헤드 없음).
//   corridor       : 소프트 회랑 셀 집합(w_corridor 페널티 면제). nullptr=미사용.
//   on_progress    : 진행율 콜백. (expanded, progress01) → bool(취소이면 true).
//                    progress01 = 1 - h_min/h_start (목표 접근도 [0, 0.99]).
//                    결과·결정성에 무영향. nullptr=미사용.
//   progress_every : 콜백 호출 간격(확장 수). 0이면 미호출.
//   in_corridor    : 하드 corridor 술어(계층 라우팅 fine A*에서 튜브 내 셀 제한).
//                    기본 AllowAll → 분기 제거, 결과 불변.
//   goal_dir       : 목표 진입 방향 제약(NEIGHBORS_6 인덱스). -1=무제약(기본, 골든 불변).
//                    ≥ 0이면 해당 방향으로 목표에 진입할 때만 성공으로 인정.
//                    막히면 호출자가 goal_dir=-1로 재시도(폴백).
template <class Occ, class InCorridor = AllowAll>
AStarResult astar_weighted(const Occ& occ, Cell start, Cell goal, const RouteParams& params,
                           long long max_expansions = -1, bool collect_visited = false,
                           const std::unordered_set<long long>* corridor = nullptr,
                           const std::function<bool(long long, double)>* on_progress = nullptr,
                           long long progress_every = 0, InCorridor in_corridor = {},
                           int goal_dir = -1,
                           const AStarTraceFn* on_trace = nullptr) {
    // ===== [0단계] 초기화 =====
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double cell_mm = params.cell_mm;

    // ===== [1단계] 시작/목표 셀 유효성 검사 =====
    // 시작이나 목표가 장애물이면 탐색 불필요 → 즉시 실패 반환(실패 사유 분류).
    if (occ.is_blocked(start)) { R.fail = RouteFail::StartBlocked; R.elapsed_ms = detail::ms_since(t0); return R; }
    if (occ.is_blocked(goal)) { R.fail = RouteFail::GoalBlocked; R.elapsed_ms = detail::ms_since(t0); return R; }
    // 계층 라우팅 fine A* 에서만 유효: start/goal이 하드 corridor 튜브 밖이면 즉시 실패.
    // 기본 AllowAll이면 항상 true → 분기 자체가 컴파일러에 의해 제거(기존 동작·골든 결과 완전 불변).
    if (!in_corridor(start) || !in_corridor(goal)) { R.fail = RouteFail::CorridorMiss; R.elapsed_ms = detail::ms_since(t0); return R; }
    // 시작==목표: 경로 길이 0, 확장 1회로 즉시 성공.
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    // ===== [2단계] 비용 모델 초기화 =====
    // CostModel: w_clear>0이면 클리어런스 전처리(Dense/Sparse=BFS, Implicit=온디맨드 설정).
    CostModel<Occ> model(occ, params, corridor);

    // ===== [3단계] 상태 공간 설정 (코너 최소직선 활성 여부) =====
    // min_straight_cells > 1 이면 상태에 run(연속 직진 셀 수)을 추가해 꺾임 금지 제약 적용.
    // RS = min_run+1: run이 [0, min_run] 범위를 가지므로 상태키 공간이 RS배 늘어난다.
    // 비활성(min_run<=1): RS=1, run=0 → 상태키 = (lin*7+(dir+1))*1+0 = lin*7+(dir+1) = 기존 동일.
    // 활성 시 탐색 시간/메모리는 ×RS. C# 기본 100mm / cell = ceil(100/cell) 셀.
    const int min_run = params.min_straight_cells;
    const bool enforce_run = min_run > 1;
    const long long RS = enforce_run ? static_cast<long long>(min_run + 1) : 1;

    // state_of — 상태 키 계산 람다.
    // 상태키 = (lin*7 + (dir+1))*RS + run.
    //   lin: 셀의 선형 인덱스(long long). Dense=int가 암묵 변환되어 오버플로 없음.
    //   dir: -1(시작)..5(NEIGHBORS_6[5]). +1해서 0..6의 7가지 값으로 만든다.
    //   run: 0..min_run(비활성이면 항상 0, RS=1로 키에 영향 없음).
    // 10mm 거대격자(20억 셀): lin~2×10^9 → lin*7~1.4×10^10 → long long 범위 내(최대 9.2×10^18).
    auto state_of = [&](long long lin, int dir, int run) -> long long {
        return (lin * 7 + (dir + 1)) * RS + (enforce_run ? run : 0);
    };
    auto trace = [&](const char* event, const Cell& from, const Cell& to,
                     long long exp, int dir, int run, int required) {
        if (on_trace) (*on_trace)(event, from, to, exp, dir, run, required);
    };

    // ===== [4단계] 탐색 자료구조 초기화 =====
    // open: 우선순위 큐(최소 힙). PQItem(f, counter, cell, dir, run).
    // (f, counter) tie-break으로 결정성 보장(동일 f → 먼저 삽입된 상태 우선).
    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;

    // nodes: 평면 해시맵(StateMap). 탐색 상태별 {g, parent, closed}를 슬롯 1개로 관리.
    // 기존 g/came/closed 세 개의 노드기반 unordered_map/set 대체 → 메모리 ~3×↓, 캐시 친화.
    detail::StateMap nodes;

    // visited_seen: collect_visited 시 셀 단위 중복 제거용(동일 셀 여러 방향 expand → 한 번만 기록).
    std::unordered_set<int> visited_seen;

    // ===== [5단계] 시작 상태 삽입 =====
    long long counter = 0;  // PQ 삽입 순서 카운터(결정성 tie-break용, 단조 증가).
    long long s0 = state_of(occ.lin(start), -1, 0);  // 시작 상태: dir=-1(방향 없음), run=0.
    { bool ins0; detail::StateMap::Slot& sl0 = nodes.emplace(s0, ins0); sl0.g = 0.0; }
    // h_start: 시작에서 목표까지의 휴리스틱(진행율 100%=0% 남음 기준).
    const double h_start = model.heuristic(start, goal);
    double h_min = h_start;  // 지금까지 확장한 셀 중 목표에 가장 가까운 휴리스틱(진행율 계산용).

    // ===== [6단계] 동적(수렴) 가중 A* 설정 =====
    // wf: 정적 가중치(w_heur). 먼 곳에서의 가중.
    // wn: 목표 근처 가중치(w_heur_near). 0이면 비활성.
    // dyn: 동적 가중 활성 여부. 무제한 탐색(max_expansions<=0)에서만 활성.
    //   이유: 예산 게이트(max_expansions>0) 탐색에서 목표 근처 가중을 낮추면
    //         탐색량이 늘어 예산 초과 → escalation/폴백 폭주(실측 cell=25: 1.3s→33s).
    //         거대격자 탐색은 항상 상한이 있으므로 자동 비활성. 골든(wn=0)도 항상 비활성.
    //
    // weighted_h(c): 셀 c에서 goal까지의 '효과적 휴리스틱'.
    //   비활성: hr * wf  (= model.heuristic와 부동소수 동일 → 골든 불변)
    //   활성:   hr * (wn + (wf-wn) * hr/h_start_raw)
    //            ↑ 먼 곳(hr ≈ h_start_raw): wf ≈ 2.0 (공격적)
    //            ↑ 목표 근처(hr → 0):       wn ≈ 1.0 (신중)
    const double wf = (params.w_heur > 0.0) ? params.w_heur : 1.0;
    const double wn = params.w_heur_near;
    const bool dyn = (wn > 0.0 && wn < wf && max_expansions <= 0);
    const double h_start_raw = model.heuristic_raw(start, goal);
    auto weighted_h = [&](const Cell& c) -> double {
        const double hr = model.heuristic_raw(c, goal);
        if (!dyn) return hr * wf;
        // 동적 가중: 거리 비율(hr/h_start_raw)로 wn ↔ wf 사이를 선형 보간.
        // hr=0(목표)이면 w=wn(최솟값), hr=h_start_raw(시작)이면 w=wf(최댓값).
        const double w = wn + (wf - wn) * (h_start_raw > 0.0 ? hr / h_start_raw : 0.0);
        return hr * w;
    };

    // 시작 상태를 open에 삽입. f = g(0) + weighted_h(start).
    open.push({weighted_h(start), counter++, start, -1, 0});
    long long expanded = 0;
    bool reached_goal_wrong_dir = false;  // goal_dir 제약 시 '목표에 닿았지만 방향 불일치' 플래그.
    bool hit_limit = false;               // 확장 상한 도달 플래그(NoPath vs ExpansionLimit 구분용).

    // ===== [7단계] A* 메인 루프 =====
    // 불변식: open이 비어 있으면 도달 가능한 미확정 상태가 없음 → 경로 없음(NoPath).
    // 매 반복: open에서 f 최솟값(동률이면 counter 최솟값) 항목을 꺼내 처리.
    while (!open.empty()) {

        // --- [7a] open에서 최적 후보 꺼내기 ---
        // top()이 f 최솟값. pop()으로 제거. (PQ는 lazy deletion 방식이므로 이미 closed인 항목이 올 수 있음)
        detail::PQItem cur = open.top();
        open.pop();

        // --- [7b] 상태 키 계산 및 closed 확인 ---
        // PQ에는 동일 상태가 여러 번 삽입될 수 있다(더 좋은 g가 발견될 때 재삽입).
        // closed=true이면 이미 최적 g로 확정된 상태 → lazy deletion으로 skip.
        long long st = state_of(occ.lin(cur.cell), cur.dir, cur.run);
        detail::StateMap::Slot* cs = nodes.find(st);  // 푸시된 state → 항상 존재.
        if (cs->closed) continue;  // 이미 확정된 상태 → skip(lazy deletion).

        // --- [7c] 상태 확정(closed로 표시) ---
        // 이 시점에서 cur의 g값이 최적임이 보장된다(A* 최적성 증명: admissible h + monotone g).
        cs->closed = true;
        const double g_st = cs->g;  // grow 전에 g값을 스택으로 복사(이후 emplace가 grow하면 cs 무효화).
        ++expanded;
        trace("expand_cell", cur.cell, cur.cell, expanded, cur.dir, cur.run, min_run);

        // --- [7d] 방문 셀 수집 (시각화용, opt-in) ---
        // collect_visited=true일 때만 실행. 동일 셀을 여러 방향으로 expand해도 한 번만 기록.
        if (collect_visited) {
            int cl = occ.lin(cur.cell);
            if (visited_seen.insert(cl).second) R.visited.push_back(cur.cell);
        }

        // --- [7e] 진행율 콜백 (협력적 취소) ---
        // progress_every 확장마다 on_progress(expanded, progress01)을 호출.
        // progress01 = 1 - h_min/h_start: 목표에 가까워질수록 1에 가까워짐(접근도).
        // 콜백이 true(취소 요청)를 반환하면 탐색을 즉시 중단(실패 결과 반환).
        // 결과·결정성에 무영향(콜백은 외부 상태 변경 없이 읽기만 함).
        if (on_progress && progress_every > 0) {
            double hc = model.heuristic(cur.cell, goal);
            if (hc < h_min) h_min = hc;  // 목표 최근접 휴리스틱 갱신.
            if (expanded % progress_every == 0) {
                double prog = (h_start > 0.0) ? 1.0 - h_min / h_start : 0.0;
                if (prog < 0.0) prog = 0.0;
                if (prog > 0.99) prog = 0.99;  // 99%로 클램프(100%=성공은 별도 반환).
                // 콜백이 취소(true)를 반환하면 탐색을 즉시 중단(실패 결과 반환) — 협력적 취소.
                if ((*on_progress)(expanded, prog)) { R.expanded_nodes = expanded; R.elapsed_ms = detail::ms_since(t0); return R; }
            }
        }

        // --- [7f] 목표 도달 검사 ---
        // cur.cell == goal이면 경로를 역추적해 반환한다.
        // goal_dir 제약(≥0): 요구된 방향(cur.dir==goal_dir)으로 진입할 때만 성공.
        //   잘못된 방향으로 goal에 도달한 상태는 closed에 추가되지만,
        //   (goal, 올바른 방향)은 다른 상태 키이므로 탐색이 계속 진행된다.
        // goal_dir<0(기본): 방향 무관 → 기존 동작·골든 불변.
        if (cur.cell == goal && goal_dir >= 0 && cur.dir != goal_dir) reached_goal_wrong_dir = true;
        if (cur.cell == goal && (goal_dir < 0 || cur.dir == goal_dir)) {
            // [경로 역추적] goal에서 parent 링크를 따라 start까지 거슬러 올라간다.
            // 상태키 s에서 셀을 추출: lin = (s / RS) / 7, occ.unlin(lin) → Cell.
            // RS=1(비활성): lin = s/7 (기존과 동일).
            std::vector<Cell> path;
            long long s = st;
            path.push_back(occ.unlin((s / RS) / 7));  // 목표 셀(goal).
            detail::StateMap::Slot* sp = nodes.find(s);
            while (sp && sp->parent != -1) {  // parent==-1이면 시작 상태 → 역추적 종료.
                s = sp->parent;
                path.push_back(occ.unlin((s / RS) / 7));
                sp = nodes.find(s);
            }
            // 역추적은 goal→start 순서 → 뒤집어 start→goal 순서로.
            for (size_t a = 0, b = path.size() - 1; a < b; ++a, --b) std::swap(path[a], path[b]);
            R.success = true;
            R.path = std::move(path);
            R.length_mm = (R.path.size() - 1) * cell_mm;  // 셀 수-1 × cell_mm (페널티 미포함).
            R.cost_mm = g_st;  // 페널티 포함 총 비용(g값) = 실제 탐색 비용.
            R.turns = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms = detail::ms_since(t0);
            return R;
        }

        // --- [7g] 확장 상한 검사 ---
        // 상한 도달 시 루프 종료. hit_limit=true로 실패 사유를 NoPath와 구분.
        if (max_expansions > 0 && expanded >= max_expansions) { hit_limit = true; break; }

        // --- [7h] 이웃 확장 (6방향 순서 고정 → 결정성 A2) ---
        // NEIGHBORS_6은 +x,-x,+y,-y,+z,-z 고정 순서(geometry.hpp).
        // 동일 비용 경로가 여럿일 때 이 순서대로 처리되어 항상 동일한 경로를 선택한다.
        const Cell* prev_off = (cur.dir < 0) ? nullptr : &NEIGHBORS_6[static_cast<size_t>(cur.dir)];
        double g_cur = g_st;  // 현재 상태의 g값(이후 emplace가 cs를 무효화할 수 있으므로 복사 사용).
        for (int nidx = 0; nidx < 6; ++nidx) {
            const Cell& d = NEIGHBORS_6[static_cast<size_t>(nidx)];
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};

            // 격자 범위 밖 또는 장애물 셀은 즉시 skip.
            if (!occ.in_bounds(nb)) {
                trace("candidate_reject_out_of_bounds", cur.cell, nb, expanded, nidx, cur.run, 0);
                continue;
            }
            if (occ.is_blocked(nb)) {
                trace("candidate_reject_blocked", cur.cell, nb, expanded, nidx, cur.run, 0);
                continue;
            }

            // 하드 corridor 제한(계층 라우팅 fine A*에서만 유효).
            // 기본 AllowAll이면 항상 true → 컴파일러가 분기 제거(성능·결과 불변).
            if (!in_corridor(nb)) {
                trace("candidate_reject_corridor_gate", cur.cell, nb, expanded, nidx, cur.run, 0);
                continue;
            }

            // --- [코너 최소직선 하드 제약] ---
            // 꺾임(방향 전환)은 현재 직진 run이 min_run 이상일 때만 허용.
            // → 모든 '꺾임~꺾임' 내부 구간 ≥ min_run셀(엘보 간 최소 단관 보장).
            // 시작 직후(dir<0)와 직진(nidx==cur.dir)은 run 제약 없음.
            // 목표 도달(위 [7f])은 run 무관하게 인정(마지막 접속 구간은 짧아도 됨).
            // 비활성(enforce_run=false): 분기 전체가 컴파일러에 의해 제거 → 완전 불변.
            int nrun = 0;
            if (enforce_run) {
                const bool is_turn = (cur.dir >= 0 && nidx != cur.dir);
                if (is_turn && cur.run < min_run) {
                    trace("candidate_reject_min_straight", cur.cell, nb, expanded, nidx, cur.run, min_run);
                    continue;
                }  // 직진 부족 → 꺾기 금지.
                // run 갱신: 꺾임 시 1로 초기화, 직진 시 min_run까지 증가 후 클램프.
                nrun = (cur.dir < 0) ? 1
                     : (nidx == cur.dir ? (cur.run < min_run ? cur.run + 1 : min_run) : 1);
            }

            // --- [이웃 g값 계산 및 갱신] ---
            long long ns = state_of(occ.lin(nb), nidx, nrun);  // 이웃 상태 키.
            double t = g_cur + model.move_cost(nb, prev_off, d);  // 잠정 g값 = g(cur) + 이동비용.

            // nodes.emplace: 이웃 상태를 맵에서 찾거나 새로 삽입.
            // grow 가능 → 반환된 sl 참조를 이후 다른 emplace/find 너머로 보관 금지.
            bool ins;
            detail::StateMap::Slot& sl = nodes.emplace(ns, ins);
            if (!ins && sl.closed) continue;  // 이미 확정된 상태 → skip.

            // 처음 방문(ins=true)이거나 더 짧은 경로 발견(t < sl.g) → g/parent 갱신 후 PQ 재삽입.
            // 기존 g가 더 좋으면 무시(A* 정확성 보장 — 같은 상태가 PQ에 여러 번 있어도 무해,
            // closed 체크([7b])로 오래된 항목은 자동 skip됨).
            if (ins || t < sl.g) {
                sl.g = t;
                sl.parent = st;  // 역추적을 위해 현재 상태를 부모로 기록.
                // f = g(이웃) + weighted_h(이웃). counter++로 삽입 순서 기록(결정성).
                open.push({t + weighted_h(nb), counter++, nb, nidx, nrun});
            }
        }
    }

    // ===== [8단계] 탐색 실패 — 실패 사유 분류 =====
    // open이 비었으나 goal에 도달하지 못함 → 실패. 사유를 3가지로 구분:
    //   ExpansionLimit: 상한 도달(어려운 경로 또는 탐색량 초과, env R3D_MAX_EXP 상향 가능).
    //   GoalDirBlocked: 목표엔 닿았으나 요구 방향으로 진입 불가(goal_dir 제약).
    //   NoPath: 연결 경로 없음(장애물로 완전 차단, 국소 혼잡).
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    // 실패 사유(A1) — 상한 도달/진입축 막힘/연결 없음 구분.
    R.fail = hit_limit ? RouteFail::ExpansionLimit
           : reached_goal_wrong_dir ? RouteFail::GoalDirBlocked
           : RouteFail::NoPath;
    return R;
}

template <class Occ, class InCorridor = AllowAll>
AStarResult astar_segmented(const Occ& occ, Cell start, Cell goal, const RouteParams& params,
                            long long max_expansions = -1, bool collect_visited = false,
                            const std::unordered_set<long long>* corridor = nullptr,
                            const std::function<bool(long long, double)>* on_progress = nullptr,
                            long long progress_every = 0, InCorridor in_corridor = {},
                            int goal_dir = -1,
                            const AStarTraceFn* on_trace = nullptr) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    if (occ.is_blocked(start)) { R.fail = RouteFail::StartBlocked; R.elapsed_ms = detail::ms_since(t0); return R; }
    if (occ.is_blocked(goal)) { R.fail = RouteFail::GoalBlocked; R.elapsed_ms = detail::ms_since(t0); return R; }
    if (!in_corridor(start) || !in_corridor(goal)) { R.fail = RouteFail::CorridorMiss; R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    CostModel<Occ> model(occ, params, corridor);
    const int min_run = params.min_straight_cells > 1 ? params.min_straight_cells : 1;
    const int max_seg = params.segment_max_cells > 0 ? params.segment_max_cells : 64;
    const bool jps_lite = params.segment_jps_lite;
    const int stride = min_run > 1 ? min_run : 4;
    auto state_of = [&](long long lin, int dir) -> long long { return lin * 7 + (dir + 1); };
    auto trace = [&](const char* event, const Cell& from, const Cell& to,
                     long long exp, int dir, int run, int required) {
        if (on_trace) (*on_trace)(event, from, to, exp, dir, run, required);
    };
    auto weighted_h = [&](const Cell& c) -> double {
        const double wf = params.w_heur > 0.0 ? params.w_heur : 1.0;
        return model.heuristic_raw(c, goal) * wf;
    };
    auto on_ray_to_goal = [&](const Cell& c, const Cell& d) -> bool {
        if (d.i != 0) return c.j == goal.j && c.k == goal.k && ((goal.i - c.i) * d.i) > 0;
        if (d.j != 0) return c.i == goal.i && c.k == goal.k && ((goal.j - c.j) * d.j) > 0;
        return c.i == goal.i && c.j == goal.j && ((goal.k - c.k) * d.k) > 0;
    };
    auto on_goal_axis_plane = [&](const Cell& c, const Cell& d) -> bool {
        if (d.i != 0) return c.i == goal.i;
        if (d.j != 0) return c.j == goal.j;
        return c.k == goal.k;
    };
    auto forced_jump_point = [&](const Cell& c, const Cell& d) -> bool {
        const Cell prev{c.i - d.i, c.j - d.j, c.k - d.k};
        for (const Cell& side : NEIGHBORS_6) {
            if ((side.i && d.i) || (side.j && d.j) || (side.k && d.k)) continue;
            Cell side_cur{c.i + side.i, c.j + side.j, c.k + side.k};
            if (!occ.in_bounds(side_cur) || occ.is_blocked(side_cur) || !in_corridor(side_cur)) continue;
            Cell side_prev{prev.i + side.i, prev.j + side.j, prev.k + side.k};
            if (!occ.in_bounds(side_prev) || occ.is_blocked(side_prev) || !in_corridor(side_prev)) return true;
        }
        return false;
    };
    auto densify = [](const std::vector<Cell>& endpoints) {
        std::vector<Cell> out;
        if (endpoints.empty()) return out;
        out.push_back(endpoints.front());
        for (size_t a = 1; a < endpoints.size(); ++a) {
            Cell p = endpoints[a - 1];
            const Cell q = endpoints[a];
            int di = (q.i > p.i) - (q.i < p.i);
            int dj = (q.j > p.j) - (q.j < p.j);
            int dk = (q.k > p.k) - (q.k < p.k);
            while (!(p == q)) {
                p = Cell{p.i + di, p.j + dj, p.k + dk};
                out.push_back(p);
            }
        }
        return out;
    };

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    detail::StateMap nodes;
    std::unordered_set<int> visited_seen;
    long long counter = 0;
    const long long s0 = state_of(occ.lin(start), -1);
    { bool ins0; detail::StateMap::Slot& sl0 = nodes.emplace(s0, ins0); sl0.g = 0.0; }
    open.push({weighted_h(start), counter++, start, -1, 0});

    const double h_start = model.heuristic(start, goal);
    double h_min = h_start;
    long long expanded = 0;
    bool hit_limit = false;
    bool reached_goal_wrong_dir = false;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        const long long st = state_of(occ.lin(cur.cell), cur.dir);
        detail::StateMap::Slot* cs = nodes.find(st);
        if (cs->closed) continue;
        cs->closed = true;
        const double g_st = cs->g;
        ++expanded;
        trace("expand_cell", cur.cell, cur.cell, expanded, cur.dir, 0, min_run);

        if (collect_visited) {
            int cl = occ.lin(cur.cell);
            if (visited_seen.insert(cl).second) R.visited.push_back(cur.cell);
        }
        if (on_progress && progress_every > 0) {
            double hc = model.heuristic(cur.cell, goal);
            if (hc < h_min) h_min = hc;
            if (expanded % progress_every == 0) {
                double prog = h_start > 0.0 ? 1.0 - h_min / h_start : 0.0;
                if (prog < 0.0) prog = 0.0;
                if (prog > 0.99) prog = 0.99;
                if ((*on_progress)(expanded, prog)) { R.expanded_nodes = expanded; R.elapsed_ms = detail::ms_since(t0); return R; }
            }
        }
        if (cur.cell == goal && goal_dir >= 0 && cur.dir != goal_dir) reached_goal_wrong_dir = true;
        if (cur.cell == goal && (goal_dir < 0 || cur.dir == goal_dir)) {
            std::vector<Cell> endpoints;
            long long s = st;
            endpoints.push_back(occ.unlin(s / 7));
            detail::StateMap::Slot* sp = nodes.find(s);
            while (sp && sp->parent != -1) {
                s = sp->parent;
                endpoints.push_back(occ.unlin(s / 7));
                sp = nodes.find(s);
            }
            for (size_t a = 0, b = endpoints.size() - 1; a < b; ++a, --b) std::swap(endpoints[a], endpoints[b]);
            R.path = densify(endpoints);
            R.success = true;
            R.length_mm = (R.path.size() - 1) * params.cell_mm;
            R.cost_mm = g_st;
            R.turns = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms = detail::ms_since(t0);
            return R;
        }
        if (max_expansions > 0 && expanded >= max_expansions) { hit_limit = true; break; }

        const Cell* prev_off0 = cur.dir < 0 ? nullptr : &NEIGHBORS_6[static_cast<size_t>(cur.dir)];
        for (int nidx = 0; nidx < 6; ++nidx) {
            const Cell& d = NEIGHBORS_6[static_cast<size_t>(nidx)];
            const bool is_turn = cur.dir >= 0 && nidx != cur.dir;
            const int min_accept = is_turn ? min_run : 1;
            const Cell* prev_off = prev_off0;
            Cell p = cur.cell;
            Cell last_valid = cur.cell;
            double seg_cost = 0.0;
            double last_valid_cost = 0.0;
            int last_valid_step = 0;
            int last_added_step = -1;
            bool blocked_before_accept = false;

            auto push_endpoint = [&](const Cell& nb, double cost_to_nb, int step, const char* reason) {
                if (step < min_accept && !(nb == goal)) return;
                const long long ns = state_of(occ.lin(nb), nidx);
                const double t = g_st + cost_to_nb;
                bool ins;
                detail::StateMap::Slot& sl = nodes.emplace(ns, ins);
                if (!ins && sl.closed) return;
                if (ins || t < sl.g) {
                    sl.g = t;
                    sl.parent = st;
                    open.push({t + weighted_h(nb), counter++, nb, nidx, 0});
                    trace(reason, cur.cell, nb, expanded, nidx, step, min_accept);
                }
                last_added_step = step;
            };

            for (int step = 1; step <= max_seg; ++step) {
                Cell nb{p.i + d.i, p.j + d.j, p.k + d.k};
                if (!occ.in_bounds(nb)) {
                    trace("candidate_reject_out_of_bounds", cur.cell, nb, expanded, nidx, step, min_accept);
                    if (jps_lite && last_valid_step >= min_accept && !(last_valid == cur.cell))
                        push_endpoint(last_valid, last_valid_cost, last_valid_step, "jump_point_ray_end");
                    break;
                }
                if (occ.is_blocked(nb)) {
                    trace("candidate_reject_blocked", cur.cell, nb, expanded, nidx, step, min_accept);
                    blocked_before_accept = step < min_accept;
                    if (jps_lite && last_valid_step >= min_accept && !(last_valid == cur.cell))
                        push_endpoint(last_valid, last_valid_cost, last_valid_step, "jump_point_ray_end");
                    break;
                }
                if (!in_corridor(nb)) {
                    trace("candidate_reject_corridor_gate", cur.cell, nb, expanded, nidx, step, min_accept);
                    if (jps_lite && last_valid_step >= min_accept && !(last_valid == cur.cell))
                        push_endpoint(last_valid, last_valid_cost, last_valid_step, "jump_point_ray_end");
                    break;
                }
                seg_cost += model.move_cost(nb, prev_off, d);
                prev_off = &d;
                p = nb;
                last_valid = nb;
                last_valid_cost = seg_cost;
                last_valid_step = step;

                const bool reaches_goal = (nb == goal);
                const bool goal_on_axis = on_ray_to_goal(cur.cell, d) && nb == goal;
                const bool goal_axis_plane = on_goal_axis_plane(nb, d);
                const bool forced = jps_lite && step >= min_accept && forced_jump_point(nb, d);
                const bool periodic = !jps_lite && step >= min_accept && ((step - min_accept) % stride == 0);
                const bool furthest_probe = step == max_seg;
                if (step >= min_accept || reaches_goal) {
                    if (periodic || reaches_goal || goal_on_axis || goal_axis_plane || forced || furthest_probe) {
                        push_endpoint(nb, seg_cost, step,
                                      forced ? "jump_point_forced" :
                                      goal_axis_plane ? "jump_point_goal_axis" :
                                      furthest_probe ? "jump_point_ray_end" : "jump_point_periodic");
                    }
                }
                if (reaches_goal) break;
            }
            if (blocked_before_accept) {
                trace("candidate_reject_min_straight", cur.cell, cur.cell, expanded, nidx, last_added_step, min_accept);
            }
        }
    }

    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    R.fail = hit_limit ? RouteFail::ExpansionLimit
           : reached_goal_wrong_dir ? RouteFail::GoalDirBlocked
           : RouteFail::NoPath;
    return R;
}
}  // namespace routing3d
