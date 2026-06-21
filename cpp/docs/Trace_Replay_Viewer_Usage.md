# Routing3D Trace Replay Viewer 사용법

- 작성일시: 2026-06-20
- 대상: Routing3D.Viewer, C++ Routing3D Engine trace log
- 로그 형식: `*.r3dtrace.jsonl`

## 1. 목적

Trace Replay Viewer는 C++ 라우팅 엔진이 남긴 탐색 로그를 다시 읽어 3D 복셀 공간에서 단계별로 검증하는 도구다.

확인할 수 있는 정보는 다음과 같다.

- 시작/목적지 셀 위치
- snap 전후 셀 위치
- A* 탐색 중 확장된 복셀
- 후보 복셀이 제외된 위치와 이유
- 후처리 전후 결과
- 최종 경로 마킹 및 작업별 성공/실패 통계

## 2. 로그 생성 방법

1. `Routing3D.Viewer`를 실행한다.
2. 상단 메뉴에서 `진단`을 연다.
3. `탐색 로그` 옵션을 켠다.
4. 자동설계를 실행한다.

`탐색 로그`가 켜진 상태에서 라우팅을 실행하면 C++ 엔진이 로그 파일을 자동으로 생성한다.

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

## 4. Trace Replay Viewer 열기

### Viewer 메뉴에서 열기

1. 상단 메뉴에서 `진단`을 연다.
2. `탐색 로그 보기`를 클릭한다.
3. Trace Replay Viewer 창이 열린다.
4. 최근 생성 로그가 있으면 자동으로 열리며, 없으면 `Open` 버튼으로 직접 선택한다.

### Trace Replay Viewer 창에서 직접 열기

1. `Open` 버튼을 클릭한다.
2. `logs` 폴더로 이동한다.
3. `*.r3dtrace.jsonl` 파일을 선택한다.

## 5. 화면 구성

| 영역 | 설명 |
|---|---|
| 상단 툴바 | 파일 열기, 재생, 이미지/동영상 저장, 필터, 레이어 토글 |
| 좌측 이벤트 목록 | JSONL 이벤트를 순서대로 표시 |
| 좌측 하단 상세 | 선택한 이벤트의 원본 JSON 표시 |
| 우측 3D 뷰 | 선택 이벤트와 관련된 복셀/경로/점유맵 표시 |

## 6. 주요 버튼

| 버튼 | 설명 |
|---|---|
| `Open` | trace 로그 파일 열기 |
| `Play` / `Pause` | 필터된 이벤트를 순차 자동 재생. `Path Playback`이 켜져 있으면 선택한 `route_path`를 셀 순서대로 누적 재생 |
| `First` | 첫 이벤트로 이동 |
| `Prev` | 이전 이벤트로 이동 |
| `Next` | 다음 이벤트로 이동 |
| `Last` | 마지막 이벤트로 이동 |
| `Fit` | 현재 표시 중인 3D 모델에 카메라 맞춤 |
| `Copy Image` | 현재 3D 뷰 이미지를 클립보드로 복사 |
| `Save Image` | 현재 3D 뷰를 PNG로 저장 |
| `Save Video` | 필터된 이벤트 재생을 영상으로 저장 |

`Speed(ms)` 슬라이더로 자동 재생 간격을 조절한다. 최소값은 10ms다.

## 7. 레이어 토글

| 토글 | 설명 |
|---|---|
| `Path Playback` | 최종 경로(`route_path`)를 시작 셀부터 종료 셀까지 순서대로 누적 표시 |
| `Voxel Map` | 전체 격자 프레임과 선택 이벤트 주변 로컬 복셀 창 표시 |
| `Occupancy Map` | 장애물/점유 셀 샘플 표시 |
| `Pass-through` | 바닥, 천장, 격자보처럼 탐색 충돌 대상은 아니지만 시각화할 통과 객체 셀 표시 |

대형 프로젝트의 점유맵은 전체 복셀을 모두 그리면 매우 무거우므로, 엔진 trace는 샘플 셀을 함께 기록하고 Viewer는 이 샘플을 표시한다.

## 8. 3D 색상 규칙

| 색상 | 의미 |
|---|---|
| Green | 시작 셀 |
| Red | 목적지 셀 |
| DeepSkyBlue | snap 후 시작 셀 |
| Yellow | snap 후 목적지 셀 |
| Gold | 확장된 복셀 |
| OrangeRed | 제외된 후보 복셀 |
| Gray | 복셀 범위 또는 점유 셀 |
| Cyan | 통과 객체 셀 |
| Bright Green | 최종 경로맵 |

## 9. 주요 이벤트 종류

| 이벤트 | 설명 |
|---|---|
| `trace_header` | 로그 기본 정보. 셀 크기, 원점, 격자 크기, 작업 수 등 |
| `occupancy_summary` | 점유 셀 요약 |
| `occupancy_sample` | 점유맵 표시용 샘플 셀 |
| `passthrough_sample` | 통과맵 표시용 샘플 셀 |
| `task_begin` | 개별 배관 라우팅 시작 |
| `snap` | 시작/목적지 셀 보정 결과 |
| `expand` | 탐색 진행률 요약 |
| `expand_cell` | 실제 확장된 복셀 |
| `candidate_reject` | 후보 복셀이 제외된 위치와 이유 |
| `postprocess` | 후처리 전후 비교 |
| `route_mark` | 최종 경로가 점유맵에 마킹된 정보 |
| `route_path` | 최종 경로 셀 목록. `Path Playback`은 이 이벤트를 순차 재생한다 |
| `task_end` | 작업 종료. 성공/실패, 길이, 꺾임, 탐색 시간 |

## 10. 후보 제외 이유

`candidate_reject` 이벤트의 `reason` 필드에서 확인한다.

| reason | 의미 |
|---|---|
| `out_of_bounds` | 후보 복셀이 격자 범위 밖 |
| `blocked` | 후보 복셀이 장애물 또는 이미 배치된 배관 점유 셀 |
| `corridor_gate` | 계층 corridor 제한 밖 |
| `min_straight` | 최소 직관 길이 조건 미달 |

예:

```json
{"type":"candidate_reject","task":12,"reason":"blocked","from":[80,35,100],"to":[81,35,100],"expanded_nodes":1000}
```

## 11. 필터 사용법

| 필터 | 설명 |
|---|---|
| `Task` | 특정 배관 번호만 표시 |
| `Type` | 특정 이벤트 종류만 표시 |
| `Find` | 이벤트 type, summary, 원본 JSON 텍스트 검색 |
| `Clear Filters` | 필터 초기화 |

추천 필터:

- `Type = task_begin`: 시작/목적지와 snap 기준 확인
- `Type = candidate_reject`: 실패 또는 우회 원인 확인
- `Find = blocked`: 장애물/배관 점유 때문에 막힌 위치 확인
- `Find = min_straight`: 최소 직관 조건 때문에 꺾임이 제한된 위치 확인
- `Type = expand_cell`: 탐색이 어느 방향으로 퍼졌는지 확인
- `Type = route_path`: 최종 경로 확인 및 `Path Playback` 재생
- `Type = task_end`: 성공/실패, 길이, 꺾임, 탐색 시간 확인

## 12. 분석 절차 예시

1. `Type = task_begin`으로 시작/목적지 셀을 확인한다.
2. `Type = snap`으로 PoC 주변 보정이 의도대로 되었는지 확인한다.
3. `Type = candidate_reject`와 `Find = blocked`로 장애물 또는 선행 배관 때문에 막힌 지점을 찾는다.
4. `Find = min_straight`로 최소 직관 제약이 경로를 얼마나 제한했는지 확인한다.
5. `Type = expand_cell`로 실제 탐색 확장 방향을 재생한다.
6. `Type = route_path`와 `Path Playback`으로 최종 경로가 시작점에서 종료점까지 어떻게 이어지는지 확인한다.
7. `route_mark`와 `task_end`에서 최종 경로, 성공 여부, 길이, 꺾임 수를 확인한다.

## 13. 로그 크기 조절

탐색 로그는 대형 프로젝트에서 매우 커질 수 있으므로 샘플링을 사용한다.

| 환경 변수 | 설명 |
|---|---|
| `R3D_TRACE_SAMPLE_EVERY` | 몇 개 확장 노드마다 상세 이벤트를 기록할지 설정 |
| `R3D_TRACE_MAX_EVENTS` | 태스크별 최대 이벤트 수 |

정밀 디버깅이 필요하면 `R3D_TRACE_SAMPLE_EVERY`를 줄인다. 단, 로그 파일 크기와 Viewer 로딩 시간이 증가한다.

## 14. 현재 한계와 다음 작업

- 옥트리 전용 탐색 경로는 trace callback 연결 범위를 추가 점검해야 한다.
- 장애물 종류별 `candidate_reject` 세분화는 아직 계획 단계다.
- 매우 큰 로그는 비동기 로딩과 가상화가 필요할 수 있다.
- Replay Viewer는 현재 이벤트 중심 표시가 기본이며, 누적 이벤트 표시 옵션은 후속 개선 대상이다.