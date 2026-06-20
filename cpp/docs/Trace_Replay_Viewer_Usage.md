# Routing3D Trace Replay Viewer 사용법

- 작성일시: 2026-06-20
- 대상: Routing3D.Viewer, C++ Routing3D Engine trace log
- 로그 형식: `*.r3dtrace.jsonl`

## 1. 목적

Trace Replay Viewer는 C++ 라우팅 엔진의 경로탐색 과정을 로그로 저장한 뒤, 해당 로그를 다시 읽어 3D 복셀맵에서 단계별로 검증하는 도구다.

다음 항목을 확인할 수 있다.

- 시작/목적지 복셀 위치
- snap 전후 복셀 위치
- A* 탐색 중 확장된 복셀
- 후보 복셀이 탈락한 위치와 이유
- 후처리 전후 결과
- 최종 라우팅 성공/실패 및 통계

## 2. 로그 생성 방법

1. `Routing3D.Viewer`를 실행한다.
2. 상단 메뉴에서 `진단`을 연다.
3. `탐색 로그` 항목을 체크한다.
4. 자동설계를 실행한다.

자동설계 실행 예:

- `기존설계`
- `다단 랙`
- 좌측 패널의 `이 그룹 전체 라우팅`
- 좌측 패널의 `이 유틸리티 전체 라우팅`

`탐색 로그`가 체크된 상태에서 라우팅을 실행하면 C++ 엔진이 탐색 로그를 자동으로 생성한다.

## 3. 로그 생성 위치

로그는 Viewer 실행 폴더 아래 `logs` 폴더에 저장된다.

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

## 4. 로그 파일 이름 규칙

로그 파일명은 대략 다음 정보를 포함한다.

```text
routing_trace_날짜_시간_프로젝트명_라벨_작업수.r3dtrace.jsonl
```

예:

```text
routing_trace_20260620_101530_WTNHJ02_existing_20tasks.r3dtrace.jsonl
```

## 5. Trace Replay Viewer 열기

### 방법 1. Viewer 메뉴에서 열기

1. 상단 메뉴에서 `진단`을 연다.
2. `탐색 로그 보기`를 클릭한다.
3. Trace Replay Viewer 창이 열린다.
4. 최근 생성 로그가 있으면 자동으로 열리거나, `Open` 버튼으로 직접 선택한다.

### 방법 2. Trace Replay Viewer에서 직접 열기

1. Trace Replay Viewer 창에서 `Open` 버튼을 클릭한다.
2. `logs` 폴더로 이동한다.
3. `*.r3dtrace.jsonl` 파일을 선택한다.

## 6. 화면 구성

Trace Replay Viewer는 크게 두 영역으로 구성된다.

### 왼쪽 이벤트 목록

| 컬럼 | 의미 |
|---|---|
| `#` | 로그 이벤트 순서 |
| `Type` | 이벤트 종류 |
| `Task` | 라우팅 작업 번호 |
| `Summary` | 이벤트 요약 |

### 오른쪽 3D 뷰

선택한 이벤트와 관련된 복셀을 3D로 표시한다.

## 7. 주요 이벤트 종류

| 이벤트 | 설명 |
|---|---|
| `trace_header` | 로그 기본 정보. 셀 크기, 원점, 격자 크기 등 |
| `occupancy_summary` | 점유 복셀 수 요약 |
| `task_begin` | 개별 배관 라우팅 시작 |
| `snap` | 시작/목적지 위치 보정 결과 |
| `expand` | 탐색 진행률 요약 |
| `expand_cell` | 실제 확장된 복셀 |
| `candidate_reject` | 후보 복셀이 제외된 위치와 이유 |
| `postprocess` | 후처리 전후 비교 |
| `route_mark` | 최종 경로가 점유맵에 마킹된 정보 |
| `task_end` | 작업 종료. 성공/실패, 길이, 꺾임 수, 탐색 시간 |

## 8. 3D 색상 의미

| 색상 | 의미 |
|---|---|
| Green | 시작 셀 |
| Red | 목적지 셀 |
| DeepSkyBlue | snap 후 시작 셀 |
| Yellow | snap 후 목적지 셀 |
| Gold | 확장된 복셀 |
| OrangeRed | 탈락한 후보 복셀 |
| DimGray | 전체 복셀맵 범위 프레임 |

## 9. 후보 탈락 이유

`candidate_reject` 이벤트의 `reason` 필드로 확인한다.

| reason | 의미 |
|---|---|
| `out_of_bounds` | 후보 복셀이 격자 범위 밖 |
| `blocked` | 후보 복셀이 장애물/점유 셀 |
| `corridor_gate` | 계층 corridor 제한 밖 |
| `min_straight` | 최소 직관 길이 조건 미달 |

예:

```json
{"type":"candidate_reject","task":12,"reason":"blocked","from":[80,35,100],"to":[81,35,100],"expanded_nodes":1000}
```

## 10. 필터 사용법

Trace Replay Viewer 상단에서 이벤트를 필터링할 수 있다.

### Task

특정 배관 번호만 보고 싶을 때 사용한다.

예:

```text
12
```

### Type

이벤트 종류별로 필터링한다.

추천 필터:

- `task_begin`: 시작/목적지 확인
- `snap`: snap 위치 확인
- `expand_cell`: 탐색 확장 방향 확인
- `candidate_reject`: 실패/우회 원인 확인
- `task_end`: 결과 통계 확인

### Find

텍스트 검색 필터다.

예:

```text
blocked
min_straight
corridor_gate
```

## 11. 재생 기능

| 버튼 | 설명 |
|---|---|
| `Play` | 필터된 이벤트를 순서대로 자동 재생 |
| `Pause` | 자동 재생 중지 |
| `First` | 첫 이벤트로 이동 |
| `Prev` | 이전 이벤트로 이동 |
| `Next` | 다음 이벤트로 이동 |
| `Last` | 마지막 이벤트로 이동 |
| `Fit` | 3D 뷰 전체 보기 |

`Speed(ms)` 슬라이더로 이벤트 간 재생 간격을 조절한다.

## 12. 추천 분석 절차

1. `Type = task_begin`으로 시작/목적지 복셀을 확인한다.
2. `Type = snap`으로 시작/목적지 보정 위치를 확인한다.
3. `Type = candidate_reject`로 후보 탈락 지점을 확인한다.
4. `Find = blocked`로 장애물 충돌이 많은지 확인한다.
5. `Find = min_straight`로 최소 직관 조건 때문에 꺾임이 제한되는지 확인한다.
6. `Type = expand_cell`로 탐색이 어느 방향으로 퍼지는지 확인한다.
7. `Play`를 눌러 필터된 이벤트를 순차 재생한다.
8. `task_end`에서 성공/실패, 길이, 꺾임 수, 탐색 시간을 확인한다.

## 13. 로그 크기 조절

탐색 로그는 대형 프로젝트에서 매우 커질 수 있으므로 샘플링을 사용한다.

환경 변수:

| 환경 변수 | 설명 |
|---|---|
| `R3D_TRACE_SAMPLE_EVERY` | 몇 개 확장 노드마다 로그를 남길지 설정 |
| `R3D_TRACE_MAX_EVENTS` | 태스크별 최대 이벤트 수 |

예:

```text
R3D_TRACE_SAMPLE_EVERY=500
R3D_TRACE_MAX_EVENTS=50000
```

값을 작게 하면 더 촘촘한 로그를 얻지만 파일 크기와 Viewer 로딩 시간이 증가한다.

## 14. 주의사항

- `탐색 로그`를 켜면 라우팅 성능이 일부 느려질 수 있다.
- 대형 프로젝트에서는 로그 파일 크기가 커질 수 있다.
- 정밀 분석이 필요한 경우에만 샘플 간격을 작게 설정하는 것이 좋다.
- 로그가 너무 크면 `Task`, `Type`, `Find` 필터를 먼저 적용해 필요한 이벤트만 확인한다.

