---
name: reference-obstacle-db
description: 장애물 데이터 PostgreSQL 접속 정보 + DDW_AI_DB TB_BIM_OBSTACLE 스키마 (Routing3D 입력 소스)
metadata:
  node_type: memory
  type: reference
  originSessionId: 12358f83-5328-4fc4-96ed-e4e7bd4740f5
---

플랜트 BIM 장애물 데이터는 PostgreSQL 에 있고 Routing3D 점유맵의 입력 소스다. **2026-06-06 DB AUTOROUTINGV7 → DDW_AI_DB 완전교체**(구 스키마 폐기). 최신 매핑은 repo `CLAUDE.md §8` 이 정답.

- **접속**: host=localhost, port=5432, dbname=`DDW_AI_DB`, user=`postgres`, password=`dinno` (로컬 dev). PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD env 우선 — 소스에 비밀번호 두지 말 것.
- **장애물 테이블**: `TB_BIM_OBSTACLE` (약 303,776행, 전 프로젝트).
  - AABB 필드: `AABB_MINX/AABB_MINY/AABB_MINZ`, `AABB_MAXX/AABB_MAXY/AABB_MAXZ` (double, **단위 mm**). (+OBB 12점)
  - 타입: `OST_TYPE`(OST_Columns 139.8k·OST_Floors 138.3k·OST_BeamStartSegment·OST_StructuralColumns·OST_StructuralFraming·OST_Ceilings·OST_Walls·빈문자 915), `DDWORKS_TYPE`, `OBS_TYPE`.
  - 신규 `COLLISION_PASS`(0/1) = **통과 가능 플래그**(직접 제공) → `ObstacleBox.PassThroughOverride`.
  - 식별: `INSTANCE_ID`(구 OBJECT_ID), `INSTANCE_NAME`(구 NAME). 위치 `POS_X/Y/Z`, 각도 `ANGLE_*`.
  - **`SOURCE_FILE` 폐지** → 프로젝트 스코프는 그룹 AABB 공간교차(아래 [[reference-routing-scene]]).
- 댐퍼(`INSTANCE_NAME` 에 'damper')는 덕트 부속이라 장애물로 로딩하지 않는다(경로 막힘 방지).
- 윈도우 한글 PG: 접속 옵션 `-c client_encoding=UTF8 -c lc_messages=C`.

**주의**: 바닥/천장(OST_Floors/Ceilings)은 건물 전체(443m×133m) 슬래브 → 전체를 50mm Dense 그리드화하면 수십억 셀. 반드시 **그룹 AABB(TB_SPACE_GROUP_INFO) 공간교차 + 격자 그룹박스 클램프**로 좁힐 것(뷰어는 장애물도 그룹 AABB 로 클리핑, 라우팅 불변).

**How to apply:** 뷰어 `Model/ObstacleDbLoader.cs`(C#, 실행 주체) 또는 Python `routing3d_py.obstacle_db`(`load_obstacles(xy_bbox=…)` / `build_occupancy`). 관련: [[project-routing3d]]
