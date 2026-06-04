# 자동경로 실행 시 기존 스텁·그룹배관 패턴 활용 — 동작 정리

> 대상: Routing3D 뷰어(C#) 자동 라우팅. 단위 mm. 기준 코드 = `csharp/Routing3D.Viewer/`.
> 작성: 2026-06-04. 본문 라인 참조는 작성 시점 기준(이후 변동 가능, 함수명으로 재확인).

이 문서는 **"자동경로를 실행할 때 사람이 설계한 기존배관(TB_ROUTE_PATH)에서 학습한 스텁과
그룹배관(번들) 패턴을 실제로 쓰는가, 쓴다면 어떻게 쓰는가"** 를 코드 기준으로 정리한다.
마지막에 결과 미니맵에서 관찰된 현상(복셀맵 미표시·점유맵 러프·방문맵↔경로맵 부분 일치)을
이 파이프라인으로 설명한다.

---

## 0. 한눈에 — 무엇을 언제 쓰나

| 자산(학습원) | 토글(필드) | 기본값 | 자동경로에서 하는 일 | 엔진 변경 |
|---|---|---|---|---|
| **스텁(수직+엘보)** | `_useStubRouting` | **ON** | 매칭 기존배관의 출발/종단 스텁을 **고정 설계 구간**으로 깔고, A\* 는 스텁 끝(랙)~끝만 탐색 | 없음(시작/끝점만 바꿈) |
| **기존설계 패턴 면(L2a)** | `_usePatterns` | **ON** | 스텁 폴백 시 PoC 를 학습된 진출/진입 면(EQUIP=−z·DUCT=+z 등)으로 표면 투영 | 없음(시작/끝점 보정) |
| **그룹배관 패턴(L4)** | `_useBundlePattern` | **ON(항상 적용)** | 같은 유틸 새 배관을 학습 **트렁크 z(공용 랙)** 에 뭉치게(`rack_levels`) + 트렁크 레인 회랑 | 파라미터(rack/corridor) |
| 기존설계 회랑(L2b) | `_useDesignCorridor` | OFF | 매칭 기존배관 폴리라인을 회랑 시드로 주입(소프트 바이어스) | 파라미터(w_corridor) |
| 유틸그룹 랙(L3a) | `_useRackBundling` | OFF | 그룹 수평 런 z-높이를 `rack_levels` 로 | 파라미터(rack) |

> **핵심**: 기본 동작(스텁 ON·패턴 ON·**그룹배관 패턴 ON**)에서 자동경로는 **스텁 라우팅 + L2a 면 투영 +
> 그룹배관 패턴(L4)** 을 항상 쓴다. 모든 패턴 활용은 **미적재/키 미스 시 자동으로 기하 규칙으로 폴백**
> 하므로 학습 데이터가 없어도 무해(회귀=실패 증가 없음).

전체 라우팅 1회는 `BuildEngineForRows(rowPositions)`
([SceneViewModel.cs:1002](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1002)) 가 엔진을 구성하고,
이어 `route_multi`(capi)가 순차 라우팅한다. 패턴 활용은 전부 이 함수 안에서 일어난다.

---

## 1. 전체 파이프라인 (BuildEngineForRows)

```
ResetEngine()
SetGrid(scene.Grid)                      ← 전체 도메인 격자(Ox,Oy,Oz,Nx,Ny,Nz)
weighted/wHeur/wHeurNear 결정            ← 격자>300k 셀이면 가중 A*(2.0)+동적수렴(near 1.0)
rackLevels = L3a(_useRackBundling)       ← 그룹 수평런 z-높이
rackLevels = MergeBundleLevels(...)      ← L4(_useBundlePattern): 번들 트렁크 z 합치기
bundleCorr  = BuildBundleCorridorCells   ← L4: 트렁크 레인 회랑 셀
wCorr 결정                               ← 회랑/랙 ON 이면 셀당 가산(0.5 또는 0.2)
SetParams(cell,500,10,2,6, wCorridor, corridorRadius=2, rackLevels, wHeur, wHeurNear)
AddObstacle / AddPassthrough             ← 장애물(통과객체는 충돌 제외)
AddFacilityObstacles                     ← 설비·덕트·이미 라우팅된 다른 배관

for each row(작업):
    ── 스텁 라우팅(_useStubRouting) ──
    stubPipe = FindMatchingExistingPipe(row)
    if stubPipe:
        (srcStub,tgtStub) = StubExtractor.ForPipe(stubPipe)
        방향 정합 → startStub/endStub 결정
        스텁 끝점(랙) = A* 시작/목표 → AddTask(스텁끝, 스텁끝)
        row.StartStub/EndStub 저장(표시·합성용)
        continue
    ── 폴백(매칭 없음/스텁 OFF): PoC 직접 ──
    startFace = LearnedFace("EQUIP",...)   ← L2a
    endFace   = LearnedDuctFace(...)       ← L2a + L3b(ANN)
    Drop/Lift/SnapPocToFreeCell(...)       ← 면 투영 + 자유셀 스냅
    AddTask(보정 시작, 보정 끝)

l2bCells = BuildDesignCorridorCells(_useDesignCorridor)   ← L2b
SetCorridorCells( CombineCorridor(l2bCells, bundleCorr) ) ← L2b ∪ L4
```

엔진은 회랑 셀(`SetCorridorCells`)과 랙 레벨(`rackLevels`)을 **`w_corridor>0` 일 때만** 비용에 반영한다.
즉 토글이 꺼져 `wCorr=0` 이면 회랑/랙 셀을 줘도 경로 모양은 표준 A\* 와 동일(무해).

---

## 2. 스텁 라우팅 (기본 ON, 가장 큰 영향)

### 2.1 동기
기존엔 PoC(장비 노즐·덕트 진입)에서 곧장 A\* 를 돌렸다. 그러면 표시용으로 그린 학습 스텁(수직배관+엘보)과
실제 A\* 경로가 **달랐다**(A\* 가 스텁을 무시하고 자기 길을 재탐색). 스텁 라우팅은 이를 일치시킨다:
**스텁을 고정 설계 구간으로 못 박고, A\* 는 스텁이 끝나는 지점(랙 위 자유공간)부터 반대쪽 스텁 끝까지만 탐색.**

### 2.2 매칭 — FindMatchingExistingPipe
([SceneViewModel.cs:2404](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L2404))
- 작업의 시작/끝 PoC 좌표 `(Sx..Gz)` 와 각 기존배관의 `SourcePos`/`TargetPos`(없으면 폴리라인 양끝)를 비교.
- 점수 = `min( d(시작,소스)+d(끝,타깃), d(시작,타깃)+d(끝,소스) )` — **양방향 허용**.
- 허용 임계 `tol×2`(tol = `max(3×cell, 1500mm)`) 이내 최소 점수 배관 채택, 초과면 매칭 없음 → 폴백.

### 2.3 스텁 추출 — StubExtractor.ForPipe
([Model/StubExtractor.cs](../csharp/Routing3D.Viewer/Model/StubExtractor.cs), Python `pattern_learn._walk_stub` 1:1 미러)
1. **방향 런 압축**(`DirRuns`): 세그먼트를 6직교 축(±x/±y/±z)으로 스냅해 연속 동일 방향 병합.
2. **지터 흡수**(`MergeShort`): `MinDirRunMm=250mm` 미만 런을 인접 런에 흡수(설계 노이즈 제거).
3. **엘보 포함**(`WalkStub`): 첫 런(수직)부터, **축이 바뀌는 첫 런(=수직→수평 엘보)** + 엘보 이후
   `LeadInMm=800mm` 리드인까지를 스텁으로 자른다. 엘보가 없으면 `MaxMm=4000mm`/`MaxBends=3` 한도.

→ 결과: 출발(장비측)·종단(덕트측) 각각 "수직배관 + 첫 엘보 + 짧은 수평 리드인" 폴리라인.

### 2.4 방향 정합 + A\* 구간 설정
([SceneViewModel.cs:1072–1095](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1072))
- 작업 start 가 배관 source 에 가까우면 정방향, 아니면 스텁 스왑(`forward`).
- 스텁 첫 점을 **실제 작업 PoC 로 고정**(표시 경로가 PoC 에서 시작/끝나도록).
- **출발 스텁 끝 = A\* 시작점**, **종단 스텁 끝 = A\* 목표점**. 두 점은 `SnapPocToFreeCell` 로 자유셀 보정.
- `AddTask(스텁끝, 스텁끝)` — 엔진은 **랙↔랙 중간 구간만** 탐색한다.

### 2.5 표시 경로 합성
([SceneViewModel.cs:2080–2090](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L2080))
```
표시 경로 = [출발 스텁] + [A* 중간 경로] + [reverse(종단 스텁)]
```
스텁은 `row.StartStub`/`row.EndStub`(월드 mm), 중간은 `row.Path`(격자 셀). 메인뷰·미니뷰 모두 이 합성을 그린다.

### 2.6 효과(실측, project6 c100 ALL 208)
- 스텁 208/208 매칭, **199→206**(랙↔랙 자유공간 탐색이 쉬움), 랙 집중도 17%→37%.
- 잔여 혼잡은 **동적 수렴 가중 A\***(`w_heur_near`)로 추가 보완 → 208/208(완전).

---

## 3. 그룹배관 패턴 — L4 (기본 ON, 항상 적용)

`_useBundlePattern` 기본 ON(항상 적용). `BundleStore`(`route_bundle_template` 적재)가 있으면 동작하고,
없으면 자동 폴백(무해). "같은 (장비·유틸) 배관들이 동일 이격간격·2회+ 꺾임으로 공유하는 공용 트렁크"를
Python `bundle_detect` 가 학습·DB 저장한 것을 읽어 신규 라우팅에 두 방식으로 주입한다.

### 3.0 어디에 키가 잡히고(동일 장비·동일 유틸) 어떻게 적용되나 — 핵심 답

**(1) 학습(탐지) 단계 — 키 = (owner_name=장비, utility)**
`bundle_detect.py` 는 기존 설계배관(TB_ROUTE_PATH)을 **(소유 장비 `SOURCE_OWNER_NAME`, 유틸리티)**
로 pre-filter 한 뒤, 그 안에서 경로 형태 유사도(형태·방향·길이·규모)로 Union-Find 군집화 →
**번들 게이트(꺾임 ≥2 + pitch 변동계수 ≤0.30)** 를 통과한 군집만 '그룹배관'으로 인정한다.
즉 **"동일 장비에서 나온 같은 유틸 배관들"이 평행 다발을 이루는 경우**를 한 그룹으로 본다.
각 그룹에서 **트렁크 고도(trunk_z, 다발이 공유하는 수평 랙 높이)** 와 **pitch(이격간격)** 를 뽑아
집계 뷰 `route_bundle_template(owner_name, utility, trunk_zs, pitch_mm, n_members)` 로 저장한다.

**(2) 적용(자동설계) 단계 — 조회 키 = utility(장비 무관 합집합)**
뷰어 `BundleStore` 는 두 인덱스를 만든다: 정확 키 `(owner,util)` 와 **유틸 폴백 `util`(그 유틸의
모든 장비 트렁크 고도 합집합)** ([BundleStore.cs:78–97](../csharp/Routing3D.Viewer/Model/BundleStore.cs#L78)).
현재 자동경로는 `_bundles.TryGet(null, util)` 로 **유틸 단위(장비 무관)** 트렁크 고도를 쓴다
([SceneViewModel.cs:1176](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1176),
[:1328](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1328)).
→ "같은 유틸의 새 배관"을 그 유틸이 기존에 쓰던 **공용 랙 고도**로 모은다(장비별로 좁히지 않음).

> 정리: **탐지는 (장비·유틸)** 로 다발을 찾지만, **적용은 유틸 단위**로 그 유틸의 학습 트렁크 고도를
> 신규 배관에 적용한다. (장비별로 더 좁혀 적용하려면 `TryGet(row의 장비명, util)` 로 바꾸면 되며,
> 정확 키 미스 시 유틸 폴백 — 현재는 행에 장비명을 싣지 않아 유틸 단위로 동작.)

**적용되는 곳(2군데)**: ① `rack_levels`(트렁크 고도 면제 z-셀 → 같은 유틸 새 배관이 그 높이로 수렴),
② 트렁크 레인 회랑(트렁크 고도 수평 런만 타이트 회랑 → 충돌회피가 인접 레인으로 분산 = 등간격 패킹).
아래 3.1·3.2 참조.

### 3.5 "기존설계 유사 (그룹배관)" 버튼 — 그룹 라우팅 모드 (2026-06-04)

자동설계를 사람 설계처럼 **그룹**으로 묶는 전용 진입점. 상단 툴바 버튼(`RerouteCorridorCommand` →
`RouteRowsAsync(AllRows(), corridor:true)`)이 `BuildEngineForRows(groupMode:true)` 로 다음을 한 번에 적용:

| 요소 | 동작 |
|---|---|
| **셀 ≤ 50mm** | `_cellMm > 50` 이면 자동으로 50mm 재적재(`LoadFromDbAsync`) — 인접 레인 분리(셀>pitch면 뭉개짐) |
| **유틸 순서** | priority `"utility"`(일반은 `"longest"`) → 같은 유틸을 묶어 순차 라우팅 → self-bundling 이 유틸별로 일관 |
| **강한 회랑** | `w_corridor = cell×2.0`(일반 0.2~0.5) → 공유 회랑 없어도 `mark_pipe`+`add_corridor_cells` 자기번들 강하게 |
| **자유공간 가이드** | `CombineCorridor(L2b 매칭 기존배관, L4 그룹 트렁크 좌표·영역)` 합집합 = 모방+그룹 패턴 |
| **트렁크 랙** | `MergeBundleLevels`(L4) 트렁크 z → `rack_levels` 유지(동적 가중 A* 도 보존) |
| **끝단** | 스텁 라우팅(기본 ON)이 매칭 기존배관 출발/종단 스텁(수직+엘보) 고정 |

→ 결과: 같은 유틸 배관이 **공용 트렁크 한 높이에 평행 다발**로 모이고, 끝단은 기존 스텁 형상. 진행
다이얼로그로 실시간 표시(`useProgress`가 그룹 모드에서도 ON). 이전의 `SetParams` 덮어쓰기 분기는 제거
(동적 가중 A*·번들 rack/corridor 가 유지되도록).

### 3.1 트렁크 고도 → rack_levels (MergeBundleLevels)
([SceneViewModel.cs:1315](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1315))
- 라우팅할 행들의 유틸 집합에 대해 번들 템플릿의 **트렁크 z(`TrunkZs`)** 를 z-셀로 변환해 `rack_levels`
  (면제 z-셀, 최대 8개)에 합친다.
- 엔진은 `rack_levels` 밖 z 에 가산(랙 페널티 cell×0.2) → 같은 유틸 새 배관이 **공용 랙 높이에 뭉친다.**

### 3.2 트렁크 레인 회랑 (BuildBundleCorridorCells)
([SceneViewModel.cs:1161](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1161))
- **레인 모드**(트렁크 z 를 알 때): 트렁크 고도 밴드(±1셀) 안의 **수평 런(랙 레인)** 만 타이트하게(±1셀) 회랑.
  - 조건: 수평(`|dz| ≤ 0.34×수평거리`) + 길이 ≥ 800mm + 트렁크 밴드 근접.
  - 좁은 레인만 깔면 `route_multi` 의 충돌회피(mark_pipe)가 새 배관을 **인접 레인에 분산** → 등간격 평행 다발로 패킹.
- 폴백(트렁크 미상): 전체 폴리라인을 넓게(±2셀) 까는 옵션1 동작.
- 회랑 셀은 `SetCorridorCells(CombineCorridor(l2b, bundleCorr))` 로 L2b 와 합쳐 엔진에 주입.

### 3.3 핵심 한계 — 셀 크기 vs pitch
**격자 셀 > 배관 이격(pitch)** 이면 인접 레인이 같은 셀로 뭉개져 물리적 패킹 불가
(예 cell=100 > pitch≈56mm). **셀 크기 ≤ pitch/2(25~50mm) 필요.** cell=50 LPS 랙집중 92% 실측.

### 3.4 표시(보조) — 그룹배관 강조
`_showBundleGroups`(별도 토글)는 라우팅이 아니라 **표시 전용**: 기존배관 중 번들 멤버를 그룹별 고유색
(`BundleGroupColor`, 황금비 색상환)으로, 비멤버는 흐리게. 자동경로 산출과는 무관.

---

## 4. 보조 패턴(요약)

| 단계 | 함수 | 역할 |
|---|---|---|
| **L2a 면 투영** | `LearnedFace`/`LiftPocToSurface` ([:1340](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1340)) | 스텁 폴백 시 PoC 를 학습된 진출/진입 면으로 표면 투영(EQUIP=−z·DUCT=+z) |
| **L3b 검색증강** | `LearnedDuctFace` ([:1349](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1349)) | 다중면 DUCT 키에서 PoC 컨텍스트 최근접(ANN) 표본으로 진입면 분기(LOO 게이트 통과 키만) |
| **L2b 회랑** | `BuildDesignCorridorCells` ([:1118](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1118)) | 매칭 기존배관 폴리라인 전체를 회랑 시드(±2셀)로 — 사람 설계 추종 |
| **L3a 랙** | `BuildRackLevels` ([:1264](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1264)) | 그룹 수평 런 z-높이를 rack_levels 로 |

모두 **미적재/키 미스 시 기하 규칙으로 자동 폴백**(무해, 회귀 0).

---

## 5. 미니맵 관찰 현상 설명

> "복셀맵 표시 안 됨 · 점유맵 러프하게 표시 · 방문맵과 경로맵이 일치하지 않지만 많은 부분 일치"

### 5.1 복셀맵(회색 선형 틀)이 안 보임
복셀맵 틀은 이제 **전체 도메인 격자 BBOX**(작은 배관 로컬이 아니라 전체)로 그린다
([:2805 BuildPipeDetail](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L2805)). `↺ 전체보기` 후 줌인
상태에서는 도메인 모서리(회색 실린더 12변)가 화면 밖이라 안 보일 수 있다 — 줌 아웃하면 보인다. (의도된 동작.)

### 5.2 점유맵이 러프함
점유맵은 전체 블록 셀을 셀-크기 적색 큐브로 그리되 **상한 150k 초과 시 균등 다운샘플**한다
([:2856 AddFullOccupancy](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L2856)). project6 c25/c10 처럼
셀 수가 많으면 솎여 **점박이(러프)** 로 보인다 — 표시 한도이지 데이터 누락이 아니다(엔진은 전수 사용).

### 5.3 방문맵 ↔ 경로맵이 "부분만 일치" — **스텁 라우팅의 직접적 결과**
이게 핵심이다. 스텁 라우팅(§2)에서 **A\* 는 스텁 끝(랙)~끝 중간 구간만 탐색**한다. 따라서:
- **방문맵(노랑)** = A\* 가 확장한 셀 = **중간 구간에만** 분포(스텁 영역엔 방문 셀이 없음).
- **경로맵(파랑)** = `[출발 스텁] + [A\* 중간] + [종단 스텁]` = **스텁까지 포함**해 방문 영역 밖으로 뻗음.
- 중간 구간에서는 경로 셀이 방문 셀의 부분집합이라 **겹치고**(많은 부분 일치), 스텁 구간에서는 방문이 없어
  경로만 존재한다(불일치). → "많은 부분 일치하지만 완전 일치는 아님"은 **정상**이며, 스텁이 동작 중이라는 증거다.

(스텁 라우팅을 끄면 A\* 가 PoC↔PoC 전체를 탐색해 방문맵이 경로 전체를 덮어 더 일치하지만, 학습 스텁
추종은 사라진다.)

---

## 6. 토글·코드 위치 빠른 참조

| UI 토글 | 필드 | 기본 | 코드 |
|---|---|---|---|
| 스텁 라우팅 | `_useStubRouting` | ON | [:217](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L217) · 적용 [:1071](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L1071) |
| 기존설계 패턴 | `_usePatterns` | ON | [:199](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L199) · `LearnedFace`/`LearnedDuctFace` |
| 그룹배관 패턴 | `_useBundlePattern` | **ON** | [:213](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L213) · `MergeBundleLevels`+`BuildBundleCorridorCells` |
| 기존설계 회랑(L2b) | `_useDesignCorridor` | OFF | [:203](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L203) · `BuildDesignCorridorCells` |
| 랙 번들(L3a) | `_useRackBundling` | OFF | [:207](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L207) · `BuildRackLevels` |
| 그룹배관 강조(표시) | `_showBundleGroups` | OFF | 표시 전용([:2140](../csharp/Routing3D.Viewer/ViewModels/SceneViewModel.cs#L2140)) |

**헤드리스 A/B 진단**: `Routing3D.Viewer.exe --dbroute <proj> <cell> ALL <out>` +
env `R3D_STUB`/`R3D_BUNDLE`/`R3D_PATTERNS`/`R3D_CORRIDOR`/`R3D_RACK`=off 로 각 단계 비교
([Diagnostics/DbRouteDiag.cs](../csharp/Routing3D.Viewer/Diagnostics/DbRouteDiag.cs)).

---

## 7. 결론

- 자동경로는 **기본적으로 스텁 라우팅(ON) + L2a 면 투영(ON)** 을 항상 사용한다. 스텁은 매칭 기존배관에서
  수직+엘보를 잘라 고정하고 A\* 는 랙↔랙만 탐색한다.
- **그룹배관 패턴(L4)은 기본 ON(항상 적용)** 이며, 트렁크 z(rack_levels)와 트렁크 레인 회랑으로
  같은 유틸 배관을 공용 랙에 등간격 패킹한다. 탐지는 (장비·유틸) 키, 적용은 유틸 단위. 단, **셀 크기 ≤
  pitch/2** 일 때만 물리적 패킹이 의미 있다(미적재/큰 셀이면 폴백, 무해).
- 모든 패턴 활용은 폴백이 있어 **회귀 0**. 미니맵의 방문↔경로 부분 일치는 스텁 라우팅의 정상 결과다.
- 관련 상세 레퍼런스: `docs/routing3d_stub_pattern.{docx,pdf}`,
  `docs/routing3d_stub_extraction.{docx,pdf}`, `docs/routing3d_bundle_detection_plan.md`,
  `docs/routing3d_pattern_learning_plan.{docx,pdf}`.
