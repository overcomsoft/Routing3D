# Routing3D Search Trace Replay 매뉴얼

- 작성일시: 2026-06-23
- 대상: `Routing3D.Viewer`, `Routing3D.TraceReplay`, C++ Routing3D Engine trace log
- 로그 형식: `*.r3dtrace.jsonl`

## 1. 목적

Routing3D Search Trace Replay 창은 C++ 라우팅 엔진이 남긴 탐색 JSONL 로그를 다시 열어, 특정 배관 태스크가 어떤 셀을 확장했고 어떤 후보 셀이 왜 거절되었으며 최종 경로가 어디로 지나갔는지 3D로 확인하는 진단 도구다.

이 창에서 확인하는 핵심 결과는 다음과 같다.

- 시작/목표 셀과 snap 보정 후 셀
- A* 또는 Segment A*가 실제로 확장한 셀
- 충돌, 범위 초과, corridor 제한, 최소 직관 조건 등으로 거절된 후보 셀
- 장애물 점유 샘플과 pass-through 샘플
- Route split, postprocess, route mark, 최종 route path
- 태스크별 성공 여부, 길이, 꺾임 수, 확장 노드 수

## 2. 로그 생성 방법

1. `Routing3D.Viewer`를 실행한다.
2. 상단 메뉴에서 `진단`을 연다.
3. `탐색 로그` 옵션을 켠다.
4. 자동설계를 실행한다.

`탐색 로그`가 켜진 상태에서 라우팅을 실행하면 C++ 엔진이 trace 파일을 생성한다. Replay 창은 이 파일을 읽기 전용으로 열며, 라우팅 중인 파일도 `FileShare.ReadWrite` 방식으로 읽을 수 있다.

## 3. 로그 저장 위치

로그는 Viewer 실행 폴더 아래 `logs` 디렉터리에 저장된다.

Release 실행 시 일반적인 위치:

```text
D:\DINNO\DEV\AI-AutoRouting\Routing3D\csharp\Routing3D.Viewer\bin\x64\Release\net9.0-windows\logs
```

Debug 실행 시 일반적인 위치:

```text
D:\DINNO\DEV\AI-AutoRouting\Routing3D\csharp\Routing3D.Viewer\bin\x64\Debug\net9.0-windows\logs
```

파일명 예:

```text
routing_trace_20260620_101530_WTNHJ02_existing_20tasks.r3dtrace.jsonl
```

## 4. Trace Replay 창 열기

### Viewer 메뉴에서 열기

1. 상단 메뉴에서 `진단`을 연다.
2. `탐색 로그 보기`를 클릭한다.
3. `Routing3D Search Trace Replay` 창이 열린다.
4. 최근 생성 로그가 있으면 자동으로 열리며, 없으면 `Open` 버튼으로 직접 선택한다.

### 독립 실행 창에서 열기

`Routing3D.TraceReplay` 프로젝트는 `TraceReplayWindow`를 단독 실행용으로 연결한다. 실행 인자로 trace 경로를 넘기면 해당 파일을 바로 열고, 인자가 없으면 `Open` 버튼으로 직접 선택한다.

### 창 안에서 직접 열기

1. `Open` 버튼을 클릭한다.
2. `logs` 폴더로 이동한다.
3. `*.r3dtrace.jsonl` 파일을 선택한다.

## 5. 화면 구성

| 영역 | 설명 |
|---|---|
| 상단 1행 툴바 | 파일 열기, 재생, 이벤트 이동, 화면 맞춤, 이미지/영상 저장 |
| 상단 2행 필터/레이어 | Task/Type/Find 필터, 레이어 표시 토글, 재생 속도 |
| 좌측 이벤트 목록 | JSONL 이벤트를 순서대로 표시. `#`, `Type`, `Task`, `Summary` 컬럼으로 구성 |
| 좌측 하단 상세 | 선택한 이벤트의 원본 JSON 한 줄을 표시 |
| 우측 3D 뷰 | 선택 이벤트와 같은 태스크의 누적 탐색 문맥, 점유 샘플, 최종 경로를 표시 |
| 우측 상단 Legend | 3D 색상 의미 |
| 상단 상태 텍스트 | 파일명, 필터 후 표시 이벤트 수, 점유/pass-through 샘플 수, cell 크기, grid 크기 |

상태 텍스트 예:

```text
routing_trace_xxx.r3dtrace.jsonl | visible 418/418 | occ=12,345 pass=900 | cell=25mm | grid=320x260x180
```

의미:

- `visible`: 현재 필터 조건으로 보이는 이벤트 수 / 전체 이벤트 수
- `occ`: 로그에 포함된 장애물/점유 샘플 셀 수
- `pass`: pass-through 샘플 셀 수
- `cell`: `trace_header.cell_mm` 값
- `grid`: `trace_header.shape` 값

## 6. 캡처 화면 기준 빠른 해석

캡처 화면은 `Task = 16`에 대해 `candidate_reject`와 `expand_cell` 이벤트가 많이 기록된 상태다. 좌측 목록에서 선택된 행은 다음과 같은 형태다.

```json
{"type":"candidate_reject","task":16,"reason":"min_straight","from":[230,201,235],"to":[230,200,235],"dir":3,"run":1,"required":12}
```

이 이벤트의 해석:

- `type=candidate_reject`: 엔진이 다음 후보 셀을 검토했지만 큐에 넣지 않았다.
- `task=16`: 16번 배관 작업의 탐색 로그다.
- `reason=min_straight`: 최소 직관 길이 조건을 만족하지 못해 거절되었다.
- `from=[230,201,235]`: 현재 셀 또는 후보 생성 기준 셀이다.
- `to=[230,200,235]`: 실제 거절된 후보 셀이다.
- `dir=3`: 이동하려던 6방향 neighbor 방향 인덱스다.
- `run=1`, `required=12`: 현재 직선 run 길이가 1셀인데 최소 12셀이 필요하다는 뜻이다.

우측 3D 화면에서는 `to` 셀이 OrangeRed 계열의 거절 후보로 강조되고, 같은 태스크에서 앞서 확장된 셀은 Gold 계열 점 구름으로 누적 표시된다. 주변의 회색 점은 장애물/점유 샘플이고, 청록 점은 pass-through 샘플이다.

## 7. 주요 버튼

| 버튼 | 설명 |
|---|---|
| `Open` | trace 로그 파일을 연다. |
| `Play` / `Pause` | 현재 필터된 이벤트 목록을 순차 자동 재생한다. |
| `First` | 필터된 첫 이벤트로 이동한다. |
| `Prev` | 필터된 이전 이벤트로 이동한다. |
| `Next` | 필터된 다음 이벤트로 이동한다. |
| `Last` | 필터된 마지막 이벤트로 이동한다. |
| `Fit` | 현재 선택 이벤트 또는 표시 중인 3D 모델에 카메라를 맞춘다. |
| `Copy Image` | 현재 3D 뷰를 클립보드 이미지로 복사한다. |
| `Save Image` | 현재 3D 뷰를 PNG로 저장한다. |
| `Save Video` | 현재 필터된 이벤트 재생을 프레임으로 저장하고, `ffmpeg`가 있으면 MP4로 인코딩한다. |

`Speed(ms)` 슬라이더는 자동 재생 간격이다. 값이 작을수록 빠르게 넘어가며, 현재 구현의 최소값은 10ms다.

## 8. 필터 사용법

| 필터 | 설명 |
|---|---|
| `Task` | 특정 배관 번호만 표시한다. 예: `16` |
| `Type` | 특정 이벤트 종류만 표시한다. 예: `candidate_reject`, `route_path` |
| `Find` | 이벤트 type, summary, 원본 JSON 전체에서 문자열 검색 |
| `Clear Filters` | Task/Find를 비우고 Type을 `All`로 되돌린다. |

추천 필터:

- `Task = 실패 태스크 번호`: 해당 배관의 탐색 흐름만 본다.
- `Type = candidate_reject`: 실패 또는 우회 원인을 본다.
- `Find = blocked`: 장애물 또는 선행 배관 점유 때문에 막힌 후보를 찾는다.
- `Find = min_straight`: 최소 직관 조건 때문에 거절된 후보를 찾는다.
- `Type = route_path`: 최종 경로를 확인하고 `Path Playback`으로 재생한다.
- `Type = task_end`: 성공/실패, 길이, 꺾임 수, 확장량을 빠르게 확인한다.
- `Type = effective_options`: 실제 적용된 Segment A*, Octree guide, Route split, expansion cap 옵션을 확인한다.
- `Type = route_split_plan`: TMT 분할의 trunk 높이와 waypoint를 확인한다.
- `Type = route_split_segment`: TruckIn/Middle/Terminal 중 어느 구간이 무겁거나 실패했는지 확인한다.

## 9. 레이어 토글

| 토글 | 기본값 | 설명 |
|---|---:|---|
| `Path Playback` | 꺼짐 | `route_path` 이벤트를 셀 순서대로 누적 재생한다. 켠 뒤 `Play`를 누르면 경로가 1셀씩 늘어난다. |
| `Voxel Map` | 켜짐 | 전체 grid 외곽 프레임, 태스크 주변 로컬 voxel window, 선택 이벤트 주변 격자선을 표시한다. |
| `Occupancy Map` | 켜짐 | `occupancy_sample` 이벤트에서 읽은 장애물/점유 샘플 셀을 표시한다. |
| `Pass-through` | 켜짐 | `passthrough_sample` 이벤트에서 읽은 통과 객체 샘플 셀을 표시한다. |

대형 scene의 점유맵 전체를 모두 그리면 매우 무겁기 때문에, Replay 창은 trace에 기록된 샘플 셀을 표시한다. 따라서 회색/청록 점은 전체 장애물의 완전한 복사본이 아니라 진단용 샘플이다.

## 10. 3D 색상과 표시 결과

| Legend | 색상 | 의미 |
|---|---|---|
| `Source cell` | LimeGreen | 원래 시작 셀 |
| `Target cell` | Red | 원래 목표 셀 |
| `Snapped source` | DeepSkyBlue | snap 후 시작 셀 |
| `Snapped target` | Yellow | snap 후 목표 셀 |
| `Expanded voxel` | Gold | 탐색에서 실제 확장된 셀 |
| `Rejected / collision candidate` | OrangeRed | 검토했지만 거절된 후보 셀 |
| `Occupancy sample` | Gray | 장애물 또는 점유맵 샘플 |
| `Pass-through sample` | Cyan | 통과 객체 샘플 |
| `Final route map` | Bright Green | 최종 경로 셀 및 경로 tube |

추가 표시:

- 진한 회색/푸른색 wireframe 큰 박스는 전체 grid 범위다.
- 선택한 태스크에 관련 셀이 있으면, 태스크 셀 범위 주변에 별도 로컬 박스가 표시된다.
- 선택 이벤트 주변에는 반경 7셀 수준의 로컬 voxel window가 표시된다.
- 같은 태스크에서 현재 선택 행 이전까지 발생한 `expand_cell`, `candidate_reject`, `snap`, `route_path`가 누적 문맥으로 함께 표시된다.
- 누적 문맥은 성능 보호를 위해 최대 약 900개 행으로 샘플링된다.

## 11. 이벤트를 선택했을 때 3D에 표시되는 결과

| 이벤트 | 좌측 Summary 예 | 우측 3D 표시 |
|---|---|---|
| `trace_header` | `cell=25 shape=[...]` | 전체 grid 크기, cell 크기 기준 설정 |
| `effective_options` | `segment=True octree=False split=True maxExp=...` | 별도 셀 강조 없음. 하단 JSON으로 실행 옵션 확인 |
| `occupancy_sample` | `sampled=1000/500000` | 회색 점유 샘플 cloud 갱신 |
| `passthrough_sample` | `sampled=500/8000` | 청록 pass-through 샘플 cloud 갱신 |
| `task_begin` | `src=[...] dst=[...] snap=[...]->[...]` | 시작/목표 셀과 snap 후 셀을 큰 큐브로 표시 |
| `snap` | `source [a]->[b]` | 보정 전 셀은 Orange, 보정 후 셀은 DeepSkyBlue로 표시 |
| `expand` | `expanded=... progress=...` | 탐색 진행률 요약. 셀 자체는 `expand_cell`에서 표시 |
| `expand_cell` | `cell=[...] exp=... dir=... run=...` | 확장된 셀을 Gold 큐브로 강조 |
| `candidate_reject` | `min_straight [from]->[to] exp=... run=1/12` | `from`은 DeepSkyBlue, 거절된 `to`는 OrangeRed 큐브로 강조 |
| `postprocess` | `unkink turns 7->3 points 476->418` | 경로 후처리 전후 통계 확인. 경로 자체는 `route_path`에서 확인 |
| `route_split_plan` | `trunkK=... z=... source=...` | 하단 JSON에서 trunk 높이와 waypoints 확인 |
| `route_split_segment` | `seg=1 [from]->[to] ok=True exp=...` | 구간별 시작/끝과 성공 여부를 JSON으로 확인 |
| `route_mark` | `path=418 radius=5` | 최종 경로가 점유맵에 마킹된 길이와 팽창 반경 확인 |
| `route_path` | `path cells=418` | 최종 경로 셀 cloud와 녹색 tube 표시. `Path Playback` 재생 대상 |
| `task_end` | `success=True len=... turns=... exp=...` | 태스크 최종 성능/성공 여부 확인 |
| `trace_limit` | `task log limit reached max=...` | 로그가 잘렸음을 의미. 이후 이벤트가 없을 수 있음 |

## 12. 주요 이벤트 JSON 필드

### `task_begin`

```json
{"type":"task_begin","task":16,"source_cell":[...],"target_cell":[...],"snapped_source":[...],"snapped_target":[...]}
```

- `source_cell`, `target_cell`: world 좌표를 grid cell로 변환한 원래 위치
- `snapped_source`, `snapped_target`: blocked 또는 부적합 위치를 주변 자유 셀로 보정한 위치

### `expand_cell`

```json
{"type":"expand_cell","task":16,"cell":[244,201,233],"expanded_nodes":295000,"dir":5,"run":1}
```

- `cell`: 실제로 closed 처리되었거나 확장된 셀
- `expanded_nodes`: 해당 시점까지의 확장 노드 수
- `dir`: 이 상태로 들어온 이동 방향
- `run`: 같은 방향으로 이어진 직선 run 길이

### `candidate_reject`

```json
{"type":"candidate_reject","task":16,"reason":"min_straight","from":[230,201,235],"to":[230,200,235],"dir":3,"run":1,"required":12}
```

- `reason`: 거절 사유
- `from`: 후보를 만든 기준 셀
- `to`: 거절된 후보 셀
- `run`: 현재 직선 run 길이
- `required`: 최소로 요구된 run 길이. `min_straight` 분석에 중요

### `route_path`

```json
{"type":"route_path","task":16,"path_points":418,"cells":[[...],[...]]}
```

- `path_points`: 최종 경로 셀 수
- `cells`: 시작부터 목표까지 이어지는 cell 좌표 배열
- `Path Playback`은 이 `cells` 배열을 앞에서부터 하나씩 늘리며 그린다.

### `task_end`

```json
{"type":"task_end","task":16,"success":true,"length_mm":10450,"turns":7,"expanded_nodes":300000,"elapsed_ms":1234}
```

- `success`: 경로 성공 여부
- `length_mm`: 최종 경로 길이
- `turns`: 꺾임 수
- `expanded_nodes`: 탐색량
- `elapsed_ms`: 해당 태스크 처리 시간

## 13. 후보 제외 이유

`candidate_reject.reason` 필드에서 확인한다.

| reason | 의미 | 주로 볼 것 |
|---|---|---|
| `out_of_bounds` | 후보 셀이 grid 범위 밖이다. | 시작/목표 또는 corridor가 scene 밖으로 밀렸는지 확인 |
| `blocked` | 후보 셀이 장애물 또는 이미 배치된 배관 점유 셀이다. | `Occupancy Map`, 선행 배관, obstacle 위치 확인 |
| `corridor_gate` | 후보 셀이 계층 corridor 또는 guide 제한 밖이다. | corridor 반경, HPA/coarse guide 품질 확인 |
| `min_straight` | 최소 직관 길이 조건을 만족하지 못했다. | `run/required`, min straight 설정값 확인 |

캡처처럼 `min_straight ... run=1/12`가 반복되면, 현재 방향 전환 후 직선 길이가 너무 짧아 후보가 계속 탈락하고 있다는 뜻이다. 이 경우 `r3d_set_min_straight_mm()` 값, 배관 관경/피팅 제약, 주변 obstacle 밀도, rack/trunk 높이를 함께 확인한다.

## 14. Path Playback 사용법

1. `Type = route_path`로 필터하거나, 분석할 태스크 행을 선택한다.
2. `Path Playback` 체크박스를 켠다.
3. `Play`를 누른다.
4. 상태 텍스트가 `Path Playback 1/418 cells | task=16`처럼 표시된다.
5. 재생이 진행되면 녹색 최종 경로가 셀 순서대로 누적된다.
6. 완료되면 `Path Playback complete 418/418 cells | task=16`이 표시된다.

`Path Playback`이 켜져 있는데 선택된 태스크에 `route_path` 이벤트가 없으면 다음 상태 메시지가 나온다.

```text
Path Playback needs a route_path event. Select a route_path row or a routed task.
```

이 메시지가 나오면 `Type = route_path` 필터를 적용하거나, 성공한 태스크의 `route_path` 행을 직접 선택한다.

## 15. 일반 분석 절차

### 실패 태스크 분석

1. `Task`에 실패한 태스크 번호를 입력한다.
2. `Type = task_begin`으로 시작/목표와 snap 위치를 확인한다.
3. `Type = candidate_reject`로 바꾸고 `Find = blocked`, `Find = min_straight` 등을 번갈아 입력한다.
4. 우측 3D에서 OrangeRed 후보가 장애물 cloud 또는 기존 경로 근처에 몰리는지 확인한다.
5. `Type = expand_cell`로 탐색이 어느 방향으로 퍼졌는지 확인한다.
6. `Type = task_end`에서 `success`, `expanded_nodes`, `elapsed_ms`를 확인한다.

### 성공 경로 품질 분석

1. `Task`를 입력한다.
2. `Type = route_path`로 최종 경로를 본다.
3. `Path Playback`으로 시작점부터 목표점까지의 진행 순서를 확인한다.
4. `Type = postprocess`에서 `turns before->after`, `points before->after`를 확인한다.
5. `Type = route_mark`에서 `path_points`와 `radius_cells`를 확인해 다음 배관에 마킹되는 점유 범위를 추정한다.

### Route Split 분석

1. `Type = effective_options`에서 `route_split=True`인지 확인한다.
2. `Type = route_split_plan`에서 `trunk_k`, `trunk_z_mm`, `source`, `waypoints`를 확인한다.
3. `Type = route_split_segment`에서 각 segment의 `success`, `fail`, `expanded_nodes`를 확인한다.
4. 특정 segment만 확장량이 크면 해당 구간의 obstacle, pass-through, corridor 조건을 집중 분석한다.

## 16. 이미지와 영상 저장

| 기능 | 결과 |
|---|---|
| `Copy Image` | 현재 3D 뷰를 Windows 클립보드에 복사 |
| `Save Image` | 현재 3D 뷰를 PNG로 저장 |
| `Save Video` | 필터된 이벤트 전체를 프레임으로 저장하고 `ffmpeg`가 있으면 MP4 생성 |

`Save Video`는 현재 필터 조건을 그대로 사용한다. 전체 로그를 필터 없이 저장하면 프레임 수가 많아질 수 있으므로, 보통은 `Task`와 `Type`을 좁힌 뒤 저장한다.

`ffmpeg`가 없거나 인코딩에 실패하면 MP4 대신 프레임 폴더가 남고 상태 텍스트에 다음과 유사하게 표시된다.

```text
Frames saved: ... (ffmpeg not found or failed)
```

## 17. 로그 크기 조절

탐색 로그는 대형 프로젝트에서 매우 커질 수 있으므로 샘플링을 사용한다.

| 환경 변수 | 설명 |
|---|---|
| `R3D_TRACE_SAMPLE_EVERY` | 몇 개 확장 노드마다 상세 이벤트를 기록할지 설정 |
| `R3D_TRACE_MAX_EVENTS` | 태스크별 최대 이벤트 수 |

정밀 디버깅이 필요하면 `R3D_TRACE_SAMPLE_EVERY`를 줄인다. 단, 로그 파일 크기와 Replay 창 로딩 시간이 증가한다. `trace_limit` 이벤트가 보이면 태스크별 이벤트 상한에 도달해 일부 상세 로그가 생략된 상태다.

## 18. 해석 시 주의사항

- `occupancy_sample`과 `passthrough_sample`은 진단용 샘플이다. 실제 전체 점유맵과 1:1로 일치하지 않을 수 있다.
- 선택한 이벤트의 같은 태스크 문맥은 현재 행 이전까지의 이벤트만 누적 표시된다. `Last`나 후반 행으로 갈수록 더 많은 탐색 흔적이 보인다.
- 누적 표시 대상이 많으면 Viewer가 일부를 샘플링한다. 전체 로그 원문은 좌측 하단 JSON과 파일에서 확인한다.
- `candidate_reject`가 많다고 반드시 실패는 아니다. A*는 많은 후보를 버리면서도 정상 경로를 찾는다. 실패 판단은 `task_end.success`, `route_path` 존재 여부, 최종 결과를 함께 본다.
- `Pass-through`는 시각화용 통과 객체 샘플이다. 일반 장애물과 다르게 충돌 차단 의미로 해석하면 안 된다.
- `Fit`은 현재 이벤트 주변 focus bounds를 우선 사용하고, focus 대상이 없으면 전체 뷰에 맞춘다.

## 19. 현재 한계와 보완 후보

- 장애물 종류별 `candidate_reject` 세분화는 아직 계획 단계다.
- `occupancy_sample`은 정적/샘플 기반이라 동적 배관 점유와 backend active voxel을 구분해서 보여주지 않는다.
- 매우 큰 로그는 비동기 로딩과 이벤트 가상화가 필요할 수 있다.
- Replay 창은 이벤트 중심 표시가 기본이며, 전체 누적 이벤트 표시 옵션은 후속 개선 대상이다.
