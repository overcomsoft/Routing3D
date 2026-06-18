// 비용함수 (Cost Function) — Routing3D C++ 엔진 (Phase 3, Step 3.4/3.6)
// =============================================================================
// [이 파일이 하는 일]
//   RouteParams(비용 가중치) + 점유맵으로 이동 비용/휴리스틱을 계산한다.
//   클리어런스는 admissibility 보호를 위해 '가산 페널티'(감산 보너스 아님).
//   명세 algorithm_spec.md §4 와 1:1 대응. 단위 mm.
//
//   Step 3.6: 점유맵 백엔드(Dense/Sparse/Vdb)에 **무관**하도록 템플릿화(헤더 정의).
//   백엔드는 동일 질의 인터페이스(in_bounds/is_blocked/lin/unlin/shape/size)만 제공하면 된다.
//
// [핵심 설계 원칙 — admissibility 보존]
//   모든 페널티는 '가산(additive)' 방식이다. 즉 이동 비용은 항상 기본(cell_mm)보다
//   크거나 같으며, 절대 작아지지 않는다(보너스·감산 금지).
//   이 덕분에 휴리스틱 h = manhattan × cell_mm 은 실제 비용의 하한(admissible)이
//   유지되고, w_heur=1.0 일 때 A* 가 최적해를 보장한다.
//   페널티 종류:
//     1) 방향 전환(꺾임) 페널티 w_turn  — 직진 선호, 불필요 꺾임 억제
//     2) 클리어런스(장애물 근접) 페널티  — 좁은 통로 기피, 관경 이격 확보
//     3) 단(z-tier) 분리 페널티 w_tier  — 특정 z 레벨 기피/유도(사용하지 않는 층 우회)
//     4) 회랑(corridor) 이탈 페널티 w_corridor — 기존설계 통로 외 셀에 추가 비용(번들링 유도)
//
// [실행 명령어 예시]
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R golden
// =============================================================================
#pragma once

#include <array>
#include <concepts>
#include <deque>
#include <map>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "routing3d/geometry.hpp"

namespace routing3d {

// HasClearanceQuery 컨셉 — 백엔드가 '온디맨드 클리어런스'를 직접 제공하는가(Step 3.11, S4).
//
// [문제] Dense/Sparse 백엔드는 전역 distance transform(clearance_map 함수)으로
//        격자 전체에 대한 '장애물까지 최단 거리' 배열을 미리 계산한다.
//        격자 크기 N에 비례하는 O(N) 메모리를 사전에 할당하므로
//        10mm 정밀 격자(수십억 셀)에서는 수십 GB가 필요해 비현실적이다.
//
// [해법] ImplicitOccupancy 는 '장애물 AABB → SpatialBoxIndex'로 색인을 유지하므로
//        임의 셀(c)에서 반경(r) 이내 장애물 유무를 O(1)~O(소수) 질의로 즉시 답한다.
//        이 컨셉을 만족하면(= clearance_cells 멤버가 있으면) CostModel 이
//        전역 배열 대신 셀별 온디맨드 질의로 대체한다.
//        → 메모리 = O(장애물 수 + 탐색 셀 수), 격자 크기 무관.
//
// Dense/Sparse 는 이 컨셉을 만족하지 않으므로 기존 배열 경로를 그대로 사용한다.
// (if constexpr 로 컴파일타임에 분기 → 런타임 비용 0)
template <class Occ>
concept HasClearanceQuery = requires(const Occ& o, const Cell& c, int r) {
    { o.clearance_cells(c, r) } -> std::convertible_to<int>;
};

// RouteParams — 라우팅 비용 파라미터 구조체
//
// A* 탐색에서 사용하는 모든 비용 관련 설정을 담는다. 단위 = mm.
// 불변식: 모든 필드 >= 0. 감산(보너스) 금지 → admissibility 보존.
// C ABI(routing3d_capi.h)의 r3d_set_* 함수들이 이 구조체의 필드를 설정한다.
// Python RouteParams(python_experiments/routing3d_py/cost.py)와 1:1 대응.
struct RouteParams {
    // 격자 셀 한 변의 실제 길이(mm). 기본 50mm.
    // 유효 범위: (0, 1000]. 작을수록 정밀도 ↑·격자 셀 수 ↑·탐색 비용 ↑.
    // 이동 1회 기본 비용(cell_mm)이자 휴리스틱 기저 단위.
    // capi: r3d_set_cell_mm / env R3D_CELL (기본 50).
    double cell_mm = 50.0;

    // 방향 전환(꺾임) 페널티(mm). 직진과 꺾임의 비용 차. 기본 500mm.
    // 유효 범위: [0, ∞). 0이면 꺾임 페널티 없음(최단 거리만 기준).
    // 높을수록 불필요한 꺾임을 강하게 억제한다 — 실측 500mm ≈ 10셀(cell=50)에 해당.
    // move_cost: prev_off != move_off 이면 cost += w_turn.
    // capi: r3d_set_w_turn.
    double w_turn = 500.0;

    // 클리어런스(장애물 근접) 페널티 가중치(mm/셀). 기본 10.0.
    // 유효 범위: [0, ∞). 0이면 클리어런스 미사용(clearance_map 미계산, 메모리 절약).
    // 실제 페널티 = w_clear × max(0, clearance_radius − d) where d = 장애물까지 거리(셀).
    // clearance_radius=2, w_clear=10 → 장애물 바로 옆 셀은 +20mm, 2셀 거리는 +10mm, 3셀 이상 0.
    // capi: r3d_set_w_clear.
    double w_clear = 10.0;

    // 휴리스틱 가중치(ε-A* / weighted A*). 기본 1.0.
    // 유효 범위: [1.0, ∞).
    //   1.0 = 표준 A*(admissible·최적·골든 불변).
    //   >1.0 = 목표 지향 탐색으로 확장 노드 급감(솔리드 장애물 우회 등 어려운 경로를
    //          탐색상한 내에 찾음). 최적해의 w_heur 배 이내 길이를 보장(준최적).
    //   실무: cell=25mm 대형 격자에서 w_heur=2.0 → 탐색량 ~10× 감소(실측).
    // capi: r3d_set_w_heur / env R3D_WHEUR.
    double w_heur = 1.0;

    // 동적(수렴) 가중 A* — 목표 근처에서 가중치를 낮춰 그리디 함정 완화.
    // 유효 범위: [0, w_heur). 0 = 비활성(정적 w_heur, 골든 불변).
    //
    // 활성(0 < w_heur_near < w_heur) 시 효과적 가중치:
    //   w_eff(c) = w_heur_near + (w_heur - w_heur_near) × (h(c) / h_start)
    //   먼 곳(h ≈ h_start): w_eff → w_heur   (공격적·빠른 탐색)
    //   목표 근처(h → 0)  : w_eff → w_heur_near (신중·최적에 가까운 탐색)
    //
    // 준최적 상한은 여전히 w_heur.
    // 주의: 예산 게이트(max_expansions > 0) 탐색에서는 자동으로 비활성화한다.
    //       (목표 근처 가중 감소 → 예산 초과 → escalation 폭주, 실측 1.3s → 33s)
    // capi: r3d_set_w_heur_near / env R3D_WHEUR_NEAR.
    double w_heur_near = 0.0;

    // 클리어런스 유효 반경(셀). 이 반경 이내에 장애물이 있으면 페널티 적용. 기본 2.
    // 유효 범위: [0, 8]. 0이면 클리어런스 미사용.
    // Dense/Sparse: clearance_map(BFS) 로 전처리. ImplicitOccupancy: 온디맨드 질의.
    // 실무: per-task 관경 라우팅(P3q B1)에서 ceil(d/cell)-1 로 설정.
    int clearance_radius = 2;

    // 클리어런스 BFS 연결성. 6(면만) 또는 26(면+모서리+꼭짓점). 기본 6.
    // 6: 더 빠름, 직교 방향 이격 평가(배관 수직 방향 회피에 충분).
    // 26: 더 정밀, 대각선 방향까지 고려(굴곡 구간 안전성 ↑).
    int clearance_connectivity = 6;

    // z셀 단(tier) 분리 페널티 맵. {z셀 인덱스 → 가산 mm}.
    // 특정 z 레벨을 기피하거나 선호 z 레벨로 유도할 때 사용.
    // 빈 map = 비활성. z셀이 맵에 없으면 페널티 0.
    // 실무: 다단 랙(P3n)에서 그룹별 전용 z-단 강제 배정에 사용.
    std::map<int, double> w_tier;

    // 회랑(corridor) 이탈 페널티(mm). 기존설계 통로 외 셀 1개당 가산. 기본 0(비활성).
    // 유효 범위: [0, ∞). >0 이면 배관 번들링 활성 — 이미 깔린 배관 옆을 따라가면 면제.
    // 0 = 비활성(corridor 집합 무시, 골든 불변).
    // capi: r3d_set_w_corridor / env R3D_WCORR.
    double w_corridor = 0.0;

    // 회랑 성장 반경(셀). 깔린 배관(셀)에서 이만큼 옆까지 회랑으로 간주. 기본 1.
    // 유효 범위: [0, 4]. 클수록 번들 허용 폭 넓어짐.
    // capi: r3d_set_corridor_radius / env R3D_CORRRAD.
    int corridor_radius = 1;

    // 선호 단(z셀 인덱스) 목록. 이 z 레벨은 회랑으로 간주 → w_corridor 페널티 면제.
    // 주 배관 랙 높이를 학습(L3a)해 넣으면 A*가 그 레벨로 자연스럽게 유도된다.
    // 빈 벡터 = 비활성.
    std::vector<int> rack_levels;

    // 코너 최소 직선(하드 제약, 절대 셀 수). 기본 0(비활성).
    // 유효 범위: [0, ∞).
    //   0 또는 1 = 비활성(상태 = (셀, 방향), 골든 바이트 완전 불변).
    //   >1 = A*가 '한 번 꺾인 뒤 이 셀 수 이상 직진하기 전엔 다시 꺾지 못하도록'
    //        상태에 진행 셀 수(run)를 추가해 하드 제약 적용.
    //        결과: 모든 내부 직선 구간(엘보 간 단관)의 길이 ≥ min_straight_cells × cell_mm.
    //        목표 도달 직전 마지막 구간은 고정 종단점 연결을 위해 길이 제약 면제.
    //   실무: capi가 ceil(min_straight_mm / cell_mm)로 mm → 셀 단위 변환해 설정.
    //         C# 기본 적용값 100mm (cell=100이면 1셀=비활성, cell=50이면 2셀=활성).
    // capi: r3d_set_min_straight_mm / env R3D_MIN_STRAIGHT_MM.
    int min_straight_cells = 0;
};

// neighbors26 — 26-연결 이웃 오프셋 배열(면+모서리+꼭짓점, 중심 제외). 고정 순서.
// clearance_map 의 connectivity=26 옵션에서 사용. 정적 초기화(첫 호출 시 1회 생성).
inline const std::array<Cell, 26>& neighbors26() {
    static const std::array<Cell, 26> n = []() {
        std::array<Cell, 26> out{};
        int m = 0;
        for (int di = -1; di <= 1; ++di)
            for (int dj = -1; dj <= 1; ++dj)
                for (int dk = -1; dk <= 1; ++dk)
                    if (di || dj || dk) out[m++] = Cell{di, dj, dk};
        return out;
    }();
    return n;
}

// clearance_map — 격자 전체에 대한 Bounded Distance Transform (BFS)
//
// 반환값: dist[lin(c)] = 셀 c에서 가장 가까운 장애물까지의 거리(셀 단위, 상한 max_radius).
//   장애물 셀 자체 = 0, 장애물 바로 옆 = 1, ..., max_radius 이상 = max_radius.
//
// [알고리즘: 다중소스 BFS(Multi-source BFS)]
//   단일소스 BFS를 N번 반복하는 O(N²) 대신, 모든 장애물 셀을 동시에 소스로 시작해
//   O(N) 시간에 전체 거리장을 계산한다(표준 distance transform).
//
//   1) 초기화: 모든 셀의 dist = max_radius.
//   2) 시딩: is_blocked(c) == true 인 셀을 dist=0으로 설정하고 BFS 큐에 삽입.
//   3) 확장: 큐에서 셀 c를 꺼내 연결성(6 또는 26) 이웃 nb를 탐사.
//            dist[nb] > dist[c] + 1 이면 dist[nb] = dist[c] + 1 로 갱신 후 큐에 삽입.
//   4) 상한: dist[c] == max_radius 이면 그 이웃은 어차피 max_radius 이므로 삽입 불필요.
//
// [시간/공간 복잡도]
//   시간: O(N × noff) — 각 셀을 최대 1회 삽입, 이웃 탐사 noff(6 또는 26)번.
//   공간: O(N) — dist 배열 + BFS 큐(최대 N 엔트리).
//
// [Dense vs Implicit 적용]
//   Dense/Sparse: CostModel 생성자에서 이 함수를 한 번 호출해 전역 배열 생성.
//   ImplicitOccupancy(HasClearanceQuery): 이 함수를 호출하지 않고 온디맨드 질의로 대체.
//
// [매개변수]
//   occ          : 점유맵 백엔드(is_blocked/in_bounds/lin/unlin/size 제공).
//   max_radius   : 계산할 최대 거리(셀). 0이면 장애물 셀만 0, 나머지 그대로.
//   connectivity : 6(면 이웃만) 또는 26(면+모서리+꼭짓점). 기본 6.
template <class Occ>
std::vector<int> clearance_map(const Occ& occ, int max_radius, int connectivity = 6) {
    if (max_radius < 0) throw std::invalid_argument("max_radius must be >= 0");
    if (connectivity != 6 && connectivity != 26)
        throw std::invalid_argument("connectivity must be 6 or 26");
    const long long N = occ.size();
    // 모든 셀의 초기 거리 = max_radius(아직 도달하지 않은 상태)
    std::vector<int> dist(static_cast<size_t>(N), max_radius);
    if (max_radius == 0) {
        // 상한이 0이면 BFS 불필요 — 장애물 셀만 0으로 표시하고 즉시 반환.
        for (long long idx = 0; idx < N; ++idx)
            if (occ.is_blocked(occ.unlin(idx))) dist[static_cast<size_t>(idx)] = 0;   // A4: 64비트 idx 직접(narrowing 제거).
        return dist;
    }

    // [1단계] 다중소스 시딩: 모든 장애물을 dist=0으로 초기화하고 큐에 넣는다.
    // BFS 출발점이 여러 개이므로 '다중소스(multi-source)' BFS라 부른다.
    std::deque<Cell> q;
    for (long long idx = 0; idx < N; ++idx) {
        Cell c = occ.unlin(idx);   // A4: long long idx 직접 전달(int 캐스팅 narrowing 제거).
        if (occ.is_blocked(c)) {
            dist[static_cast<size_t>(idx)] = 0;
            q.push_back(c);
        }
    }
    // 이웃 오프셋 배열 선택(연결성에 따라 6 또는 26방향)
    const Cell* off = (connectivity == 26) ? neighbors26().data() : NEIGHBORS_6.data();
    const int noff = (connectivity == 26) ? 26 : 6;

    // [2단계] BFS 확장: 거리를 장애물에서 밖으로 1씩 늘리며 전파.
    // 표준 BFS이므로 셀은 최단 거리로 처음 방문될 때 확정된다(최적성 보장).
    while (!q.empty()) {
        Cell c = q.front();
        q.pop_front();
        int d = dist[static_cast<size_t>(occ.lin(c))];
        // 이미 상한에 도달한 셀의 이웃은 어차피 max_radius → 삽입 불필요(조기 종료).
        if (d >= max_radius) continue;
        for (int t = 0; t < noff; ++t) {
            Cell nb{c.i + off[t].i, c.j + off[t].j, c.k + off[t].k};
            if (!occ.in_bounds(nb)) continue;
            size_t nl = static_cast<size_t>(occ.lin(nb));
            // 더 짧은 경로가 발견됐을 때만 갱신(이미 방문한 셀도 더 짧으면 재삽입).
            // BFS이므로 실제로는 각 셀이 최초 방문 시 최적 거리로 확정되어 재삽입은 거의 없음.
            if (dist[nl] > d + 1) {
                dist[nl] = d + 1;
                q.push_back(nb);
            }
        }
    }
    return dist;
}

// CostModel — 이동 비용·휴리스틱 계산기
//
// 점유맵 백엔드(Occ)와 RouteParams를 받아 A*가 요청하는 두 가지 값을 계산한다:
//   1) move_cost(to, prev_off, move_off) — 이동 1회의 실제 비용 g 증분
//   2) heuristic(c, goal)               — 남은 거리의 하한 추정치(admissible)
//
// [비용 구성 요약]
//   move_cost = cell_mm                          (기본 이동 1셀)
//             + w_turn  (if 방향 전환)           (꺾임 페널티)
//             + w_clear × max(0, r - d_clear)   (장애물 근접 페널티)
//             + w_tier[c.k]  (if 해당 z레벨)    (단 분리 페널티)
//             + w_corridor   (if 회랑 이탈)      (번들링 유도 페널티)
//
// [admissibility 보존]
//   모든 항이 >= 0 이므로 move_cost >= cell_mm.
//   heuristic = manhattan × cell_mm ≤ 실제 이동비용(페널티 제외 하한).
//   → w_heur=1.0 이면 A*가 항상 최적해를 반환함이 보장된다.
//
// 백엔드 무관(템플릿). 클리어런스 비활성이면 distance transform 미계산.
template <class Occ>
class CostModel {
public:
    // 생성자 — 클리어런스 전처리 또는 온디맨드 설정.
    //
    // corridor: 회랑 셀 집합(lin 인덱스 → long long). nullptr이면 회랑 미사용(rack_levels만 면제).
    //   키가 long long인 이유: ImplicitOccupancy의 lin()이 64비트(거대 격자 20억 셀 이상).
    //   Dense/Sparse의 lin()은 int이지만 long long으로 암묵 변환돼 무손실.
    //
    // [클리어런스 초기화 전략(컴파일타임 분기)]
    //   HasClearanceQuery<Occ> == true  (ImplicitOccupancy):
    //     → on_demand_clear_ = true. clearance_ 배열 미생성. 셀 질의 시마다 occ_.clearance_cells() 호출.
    //       메모리 O(1)(배열 없음), 질의 비용 O(장애물 수 / 격자 밀도).
    //   HasClearanceQuery<Occ> == false (Dense/Sparse):
    //     → clearance_map() BFS로 전체 격자 거리 배열 생성. 메모리 O(N). 이후 질의 O(1).
    CostModel(const Occ& occ, RouteParams params,
              const std::unordered_set<long long>* corridor = nullptr)
        : occ_(occ), p_(std::move(params)), corridor_(corridor) {
        if (p_.w_clear > 0.0 && p_.clearance_radius > 0) {
            if constexpr (HasClearanceQuery<Occ>) {
                on_demand_clear_ = true;  // 백엔드 질의로 대체 — 전역 배열 미생성(S4).
            } else {
                // Dense/Sparse: BFS distance transform 전처리(O(N) 시간/공간).
                clearance_ = clearance_map(occ_, p_.clearance_radius, p_.clearance_connectivity);
                has_clear_ = true;
            }
        }
        // rack_levels 벡터를 unordered_set으로 캐시(cell_penalty에서 O(1) 조회).
        rack_ = std::unordered_set<int>(p_.rack_levels.begin(), p_.rack_levels.end());
    }

    // cell_penalty — 셀 c에 도달할 때의 '위치 기반' 가산 페널티(mm).
    //
    // [페널티 항목]
    //   1) 클리어런스 페널티:
    //        d = 셀 c에서 가장 가까운 장애물까지의 거리(셀).
    //        pen += w_clear × max(0, clearance_radius - d)
    //        → 장애물에 가까울수록 높은 페널티(관경 이격 유도).
    //   2) z-tier 페널티:
    //        pen += w_tier[c.k]  (c.k가 w_tier 맵에 있으면)
    //        → 특정 z 레벨 기피 또는 선호 레벨 유도.
    //   3) 회랑 이탈 페널티:
    //        on_corridor = (c.k ∈ rack_levels) OR (lin(c) ∈ corridor_)
    //        if !on_corridor: pen += w_corridor
    //        → '회랑 밖' 셀에 가산(보너스가 아님 → admissibility 보존).
    //        회랑 = 이미 깔린 배관 ±corridor_radius 셀 + rack_levels z레벨.
    double cell_penalty(const Cell& c) const {
        double pen = 0.0;
        // [클리어런스 페널티] — 컴파일타임 분기(HasClearanceQuery)
        if constexpr (HasClearanceQuery<Occ>) {
            if (on_demand_clear_) {
                // ImplicitOccupancy: 온디맨드 질의(배열 없이 장애물 색인에서 즉시 계산)
                int d = occ_.clearance_cells(c, p_.clearance_radius);
                if (d < p_.clearance_radius) pen += p_.w_clear * (p_.clearance_radius - d);
            }
        } else if (has_clear_) {
            // Dense/Sparse: 전처리된 배열에서 O(1) 조회
            int d = clearance_[static_cast<size_t>(occ_.lin(c))];
            if (d < p_.clearance_radius) pen += p_.w_clear * (p_.clearance_radius - d);
        }
        // [z-tier 페널티] — 특정 z 레벨에 추가 비용(다단 랙 강제 배정에 사용)
        if (!p_.w_tier.empty()) {
            auto it = p_.w_tier.find(c.k);
            if (it != p_.w_tier.end()) pen += it->second;
        }
        // [회랑 이탈 페널티] — 번들링 유도(기존설계 통로를 따라 배관)
        // 주의: '회랑 안' 셀은 면제(페널티 0), '회랑 밖' 셀에 가산 → admissibility 보존.
        // Python cost.py cell_penalty와 1:1 대응.
        if (p_.w_corridor > 0.0) {
            // 회랑(이미 깔린 배관 곁) 또는 선호 단(rack)에 속하면 면제, 아니면 가산.
            // Python cost.py cell_penalty 와 1:1. 보너스가 아닌 '회랑 밖 가산'이라 admissibility 보존.
            bool on_corridor = (rack_.find(c.k) != rack_.end()) ||
                (corridor_ != nullptr &&
                 corridor_->find(static_cast<long long>(occ_.lin(c))) != corridor_->end());
            if (!on_corridor) pen += p_.w_corridor;
        }
        return pen;
    }

    // move_cost — 셀 하나를 이동할 때의 실제 비용 g 증분(mm).
    //
    // [매개변수]
    //   to       : 이동 목적지 셀
    //   prev_off : 이전 이동 방향 오프셋. nullptr이면 시작 첫 이동(회전 없음).
    //   move_off : 현재 이동 방향 오프셋
    //
    // [비용 계산]
    //   기본 비용 = cell_mm (1셀 이동)
    //   + w_turn  if prev_off != nullptr AND prev_off != move_off  (방향 전환)
    //   + cell_penalty(to)  (위치 기반 가산 페널티)
    double move_cost(const Cell& to, const Cell* prev_off, const Cell& move_off) const {
        double c = p_.cell_mm;
        if (prev_off != nullptr && !(*prev_off == move_off)) c += p_.w_turn;
        c += cell_penalty(to);
        return c;
    }

    // heuristic — A*의 남은 거리 추정(admissible 하한). 가중 맨해튼 거리(mm × w_heur).
    // 모든 페널티를 0으로 가정한 하한이므로 실제 최적 비용 ≤ h × (1/w_heur)가 보장된다.
    // 동적 가중의 기저로도 사용(heuristic_raw 참조).
    double heuristic(const Cell& c, const Cell& goal) const {  // manhattan × cell_mm × w_heur
        const double w = (p_.w_heur > 0.0) ? p_.w_heur : 1.0;
        return manhattan(c, goal) * p_.cell_mm * w;
    }

    // heuristic_raw — 가중 없는 맨해튼 휴리스틱(mm). w_heur 미적용.
    // 동적 가중 A*에서 astar_weighted가 거리 비율로 가중치를 보간할 때 사용.
    // (hr / h_start_raw) 비율로 현재 위치가 목표에 얼마나 가까운지 측정.
    double heuristic_raw(const Cell& c, const Cell& goal) const {
        return manhattan(c, goal) * p_.cell_mm;
    }

private:
    const Occ& occ_;
    RouteParams p_;
    // Dense/Sparse용 전역 클리어런스 배열. 비어 있으면 클리어런스 비활성 또는 온디맨드 모드.
    std::vector<int> clearance_;
    // Dense/Sparse에서 clearance_ 배열이 유효한가.
    bool has_clear_ = false;
    // ImplicitOccupancy에서 온디맨드 질의를 쓰는가(배열 없이 occ_.clearance_cells 호출).
    bool on_demand_clear_ = false;
    // 회랑 셀 집합(lin → long long). nullptr이면 회랑 미사용(rack_levels만 면제).
    const std::unordered_set<long long>* corridor_ = nullptr;
    // rack_levels의 빠른 O(1) 조회용 캐시(Python frozenset 대응).
    std::unordered_set<int> rack_;
};

}  // namespace routing3d
