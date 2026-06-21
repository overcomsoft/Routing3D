# WTNHJ02 CLEAN Exhaust 20개 배관 자동경로 6방식 비교 리포트

- 작성일시: 2026-06-20
- 대상: DDW_AI_DB project 1 / WTNHJ02 / CLEAN 장비 주변 Exhaust 그룹 20개 배관
- 실행 명령: `dotnet Routing3D.Viewer.dll --dbroute 1 25 Exhaust <out>`
- 격자: 25mm
- 결과 폴더: `docs/route_compare_wtnhj02_exhaust_20260620_133720/`
- 주의: S6는 실제 실행 9분 이상 지속되어 시간초과로 수동 종료했다.

## 1. 요약

- 가장 빠른 완료 전략: **S4 Existing-design corridor**, 3.29초
- 최단 총길이 전략: **S5 Learned rack + bundle**, 67,650mm
- 최소 꺾임 전략: **S5 Learned rack + bundle**, 31 turns
- 순수/기본 A* 계열(S1/S2)은 90초 이상 걸렸고, 스텁 기반 전략(S3~S5)은 3.3~3.5초 수준으로 크게 개선됐다.
- S3/S5는 길이와 꺾임이 동일했고, S4는 회랑 바이어스 때문에 꺾임이 1개 증가했지만 bundle 밀집 지표는 더 낮았다.

## 2. 정량 비교

| ID | 방식 | 상태 | 성공 | 시간(s) | 총길이(mm) | 길이 변화 | 꺾임 | 꺾임 변화 | 평균길이(mm/배관) | 평균꺾임 | rackZ | 비고 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| S1 | Weighted A* baseline | 완료 | 20/20 | 92.96 | 315,175 | 0.0% | 147 | 0.0% | 15,759 | 7.35 | 0.0% | - |
| S2 | PoC snap + learned face | 완료 | 20/20 | 91.60 | 315,100 | 0.0% | 149 | +1.4% | 15,755 | 7.45 | 0.0% | - |
| S3 | Stub endpoints + A* | 완료 | 20/20 | 3.53 | 67,650 | -78.5% | 31 | -78.9% | 3,383 | 1.55 | 93.1% | [stub 20/20] |
| S4 | Existing-design corridor | 완료 | 20/20 | 3.29 | 67,650 | -78.5% | 32 | -78.2% | 3,383 | 1.6 | 88.8% | [stub 20/20] |
| S5 | Learned rack + bundle | 완료 | 20/20 | 3.31 | 67,650 | -78.5% | 31 | -78.9% | 3,383 | 1.55 | 93.1% | [stub 20/20] |
| S6 | Follow existing + local repair | 시간초과 | - | - | - | - | - | - | - | - | - | 9분 이상 시간초과, 출력 미생성 |

## 3. 방식별 장단점

| ID | 방식 | 장점 | 단점 | 판단 |
|---|---|---|---|---|
| S1 | Weighted A* baseline | 기능 의존성이 적고 순수 엔진 성능 기준선으로 좋다. 모든 배관 20/20 성공. | 25mm 격자에서 92.96초로 느리고, 총길이/꺾임이 크다. Trace에서 보듯 탐색이 넓게 산재한다. | fallback 기준선으로만 유지 권장. |
| S2 | PoC snap + learned face | PoC가 장애물 내부에 묻히는 문제를 줄이고 실제 접속면을 반영한다. | 이번 케이스에서는 S1 대비 시간/길이/꺾임 개선이 거의 없다. 핵심 병목인 셀 탐색량은 그대로다. | 단독 품질개선 효과는 제한적. 다른 전략의 전처리로 가치가 있다. |
| S3 | Stub endpoints + A* | 시간 3.53초, 길이 67,650mm, 31 turns로 급격히 개선. 기존 설계의 접속 스텁을 활용해 탐색 범위를 줄인다. | 기존배관 매칭/스텁 추출 품질에 의존한다. 신규 설계 또는 매칭 실패 배관에서는 효과가 줄 수 있다. | 이번 케이스 최우선 실전 전략. |
| S4 | Existing-design corridor | S3와 비슷한 속도이며 기존설계 회랑을 따라가도록 유도한다. bundle 밀집 지표가 가장 낮게 관찰됐다. | S3 대비 꺾임이 1개 증가했다. 회랑 바이어스가 강하면 불필요한 추종/우회가 생길 수 있다. | 설계 유사도/다발 정렬을 중시할 때 선택. |
| S5 | Learned rack + bundle | S3와 같은 길이/꺾임, 3.31초. rackZ 93.1%로 학습 rack 높이를 강하게 따른다. | 이번 Exhaust 단일 그룹에서는 S3 대비 추가 이득이 작다. 다그룹/혼잡 케이스에서 효과가 더 클 가능성이 높다. | 다그룹 라우팅의 기본 후보. 단일 Exhaust는 S3와 동급. |
| S6 | Follow existing + local repair | 성공하면 사람 설계 형상을 가장 직접적으로 보존할 수 있다. | 실제 실행에서 9분 이상 지속되어 시간초과. local repair 비용이 크거나 종료/상한 관리가 부족하다. | 현재 상태로는 대량 배치 기본 전략 부적합. timeout/cap/구간 제한 필요. |

## 4. 결론

이번 WTNHJ02 CLEAN Exhaust 20개 실측 비교에서는 **스텁 기반 A***가 기존 셀 기반 A* 대비 압도적으로 좋았다.

- S1 baseline: 92.96초, 315,175mm, 147 turns
- S3 stub: 3.53초, 67,650mm, 31 turns

즉 이 케이스에서는 “전체 3D 셀 공간을 처음부터 찾는 방식”보다 “기존설계/스텁에서 접속 구조를 먼저 얻고 짧은 구간만 A*로 연결하는 방식”이 속도와 품질 모두 우세하다.

추천 운영안:

1. Exhaust 단일 그룹: S3 또는 S5를 우선 사용한다.
2. 다그룹/혼잡 구간: S5 rack+bundle을 우선 적용한다.
3. 기존설계 유사도가 중요한 검토 모드: S4 corridor를 병행 비교한다.
4. S6 복제 방식은 timeout, repair 구간 길이 제한, task별 상한을 넣기 전까지 대량 자동설계 기본값으로 쓰지 않는다.
5. 다음 엔진 개선은 cell 단위 A*보다 segment 단위 A* 또는 macro-corridor + segment routing 쪽이 타당하다.

## 5. 원시 결과

### S1 Weighted A* baseline

```text
G route_multi +facilities+drop clearON(Implicit ?⑤뵒留⑤뱶): success 20/20 totalLen 315175 turns 147 (92958 ms) [progress cb 20, fail 0] rackZ=0.0% (z? 176,177) 踰덈뱾諛吏?3.4%
```

### S2 PoC snap + learned face

```text
G route_multi +facilities+drop clearON(Implicit ?⑤뵒留⑤뱶): success 20/20 totalLen 315100 turns 149 (91602 ms) [progress cb 20, fail 0] rackZ=0.0% (z? 176,177) 踰덈뱾諛吏?3.5%
```

### S3 Stub endpoints + A*

```text
G route_multi +facilities+drop clearON(Implicit ?⑤뵒留⑤뱶): success 20/20 totalLen 67650 turns 31 (3528 ms) [progress cb 20, fail 0] rackZ=93.1% (z? 176,177) [stub 20/20] 踰덈뱾諛吏?1.2%
```

### S4 Existing-design corridor

```text
G route_multi +facilities+drop clearON(Implicit ?⑤뵒留⑤뱶): success 20/20 totalLen 67650 turns 32 (3291 ms) [progress cb 20, fail 0] rackZ=88.8% (z? 176,177) [stub 20/20] corridor=145096? wCorr=13 踰덈뱾諛吏?0.9%
```

### S5 Learned rack + bundle

```text
G route_multi +facilities+drop clearON(Implicit ?⑤뵒留⑤뱶): success 20/20 totalLen 67650 turns 31 (3305 ms) [progress cb 20, fail 0] rackZ=93.1% (z? 176,177) [stub 20/20] 踰덈뱾諛吏?1.2%
```

### S6 Follow existing + local repair

```text
S6 실행은 9분 이상 지속되어 수동 종료. 출력 파일 미생성.
```

