# Routing3D.AutoRouteViewer

AutoRouteModule 검증 전용 WPF 3D Viewer입니다. 기존 Routing3D Viewer의 화면 흐름을 참고해 상단 DB 접속 바, 프로젝트 선택, 좌측 유틸리티 라우팅 패널, 중앙 3D overlay, 우측 자동설계 결과/분석 패널로 구성했습니다.

## 화면 구성

- 상단: PostgreSQL Host/Port/User/Password/Database, 프로젝트 목록/로드, 전체 라우팅, 취소
- 좌측: 유틸리티 그룹, 유틸리티 필터, 선택 경로 라우팅, 로드된 객체 수 표시
- 중앙: HelixToolkit 3D overlay
  - 주황 튜브: PostgreSQL 기존 설계 경로 전체(`TB_ROUTE_PATH`/`TB_ROUTE_SEGMENT_DETAIL`, 데이터 로딩 직후 표시)
  - 시안 튜브: AutoRouteModule 신규 탐색 결과
  - 회색/빨강 계열 박스: 장애물 AABB
  - 주황 박스+프레임: 설비/메인 설비
  - 파랑 박스+프레임: 덕트
  - 초록 박스+프레임: 레터럴
  - 빨강 구: 시작 PoC
  - 파랑 구: 종단 PoC
  - 노랑 구: 배관 부속
- 우측: 자동설계 결과 경로, 분석결과, 단계별 경로

## PostgreSQL 연동

기본 접속값은 화면 상단에서 직접 입력합니다.

- Host: `localhost`
- Port: `5432`
- User: `postgres`
- Password: `dinno`
- Database: `DDW_AI_DB`

환경변수도 지원합니다.

- `PGHOST`
- `PGPORT`
- `PGUSER`
- `PGPASSWORD`
- `PGDATABASE`

## 프로젝트 목록

`목록` 버튼은 `TB_SPACE_GROUP_INFO`에서 프로젝트를 읽습니다.

```sql
SELECT "TAG_GROUP_ID","TAG_GROUP_NM","BAY_GROUP_NM","PROCESS_GROUP_NM",
       "AABB_MINX","AABB_MINY","AABB_MINZ","AABB_MAXX","AABB_MAXY","AABB_MAXZ"
FROM "TB_SPACE_GROUP_INFO"
ORDER BY "PROCESS_GROUP_NM","TAG_GROUP_NM";
```

## 기존 Viewer 방식 데이터 로드

`로드` 버튼은 신규 프로젝트 내부의 `LegacyDb.ObstacleDbLoader` 구현을 호출합니다. 이 구현은 기존 Viewer의 실제 DB 로딩 흐름을 신규 프로젝트 내부 소스로 이식한 것입니다. 선택 프로젝트의 `TB_SPACE_GROUP_INFO` AABB에 500 mm margin을 더한 범위로 다음 데이터를 읽습니다.

- 장애물: `TB_BIM_OBSTACLE`
- 설비: `TB_EQUIPMENTS`
- 덕트: `TB_DUCT`
- 레터럴: `TB_LATERAL_PIPE`
- 공간: `TB_SPACE_INFO`
- 기존 설계 경로 및 작업 PoC: `TB_ROUTE_PATH`, `TB_ROUTE_SEGMENTS`, `TB_ROUTE_SEGMENT_DETAIL`
- 설비/덕트/레터럴 PoC 후보: `TB_POCINSTANCES`
- 배관 부속: `TB_ROUTE_SEGMENT_DETAIL` 중 `PIPE`, `POC`, `BENDING`을 제외한 타입

자동검색의 장애물 입력은 신규 프로젝트 내부 `LegacyDb` 로더가 만든 `SceneData`를 신규 Viewer 모델로 변환한 뒤, `TB_BIM_OBSTACLE`의 collision-pass 제외 solid, 설비, 덕트, 레터럴을 합산한 `RoutingSolids`입니다. 따라서 기존 뷰어처럼 설비와 덕트/레터럴을 회피 대상으로 보면서 자동경로를 탐색합니다.

## 실행 순서

1. DB 접속값 입력
2. `목록` 클릭
3. 프로젝트 선택
4. `로드` 클릭
5. 좌측 객체 수에서 설비/덕트/PoC/검색솔리드 로드 상태 확인
6. 유틸리티 그룹/유틸리티 필터 선택
7. `선택 경로 라우팅`, `이 그룹 전체 라우팅`, 또는 `전체보기` 실행

## 빌드

```powershell
dotnet build D:\DINNO\DEV\AI-AutoRouting\Routing3D\csharp\Routing3D.Viewer.sln -c Debug -p:Platform=x64
```
