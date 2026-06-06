---
name: reference-routing-scene
description: DDW_AI_DB 라우팅 씬 스키마 (그룹=툴 → 장애물/장비/PoC/작업·기존배관/유틸리티)
metadata:
  node_type: memory
  type: reference
  originSessionId: 12358f83-5328-4fc4-96ed-e4e7bd4740f5
---

라우팅 입력 씬은 **DDW_AI_DB** 에서 **그룹(툴) 단위**로 구성한다. **2026-06-06 AUTOROUTINGV7 완전교체**(구 space_project_map·POC_LIST jsonb 스키마 폐기). 최신·정본은 repo `CLAUDE.md §8`.

- **프로젝트=그룹(툴) 식별**: `TB_SPACE_GROUP_INFO`(TAG_GROUP_ID, TAG_GROUP_NM[예 WTNHJ02], BAY_GROUP_NM, PROCESS_GROUP_NM, AABB_MIN/MAX_*). 1-based 순번으로 콤보/`--dbroute`. 구 `SOURCE_FILE` 없음 → **그룹 AABB ±500mm 공간교차**로 모든 객체 스코프.
- **장애물**: `TB_BIM_OBSTACLE`(AABB_*·COLLISION_PASS, damper 제외) → 점유맵. [[reference-obstacle-db]]
- **장비**: `TB_EQUIPMENTS`(`MAIN_SUB_TYPE`='MainTool'=메인, AABB_*, INSTANCE_NAME). PoC 는 별도 `TB_POCINSTANCES`(INSTANCE_ID·OWNER_INSTANCE_ID·OWNER_INSTANCE_TYPE·UTILITY_NM·PIPESIZE_NM·POSX/Y/Z).
- **작업·기존배관(정본=route_path)**: `TB_ROUTE_PATH` 1행 = 작업 1개(엔드포인트) + 기존 설계배관 1개(폴리라인). 구 POC_LIST jsonb 페어링을 대체.
  - 작업 = `SOURCE_POS`→`TARGET_POS`. 출발명=`EQUIPMENT_NAME`, 종단명=`TARGET_OWNER_NAME`, 유틸=`SOURCE_UTILITY`, 그룹=`UTILITY_GROUP`, 관경=`SOURCE_SIZE`. PoC 실체 = `SOURCE_GUID`/`TARGET_GUID` → `TB_POCINSTANCES.INSTANCE_ID`(1:1).
  - 폴리라인 = `TB_ROUTE_SEGMENT_DETAIL`(FROM/TO_POS) ⨝ `TB_ROUTE_SEGMENTS`(ORDER) ⨝ `TB_ROUTE_PATH`. 스코프 rp.SOURCE_POS in 그룹박스.
  - 배관 자재(부속) = `TB_ROUTE_SEGMENT_DETAIL.TYPE`(PIPE/POC/BENDING 제외 = ELBOW/TEE/VALVE/FLANGE…).
  - **종단 주의**: `TARGET_OWNER_NAME` 이 Duct 가 아니라 Damper/Elbow/Takeoff 일 수 있음(설계상 배기 배관이 덕트 부속에 접속, 정상). 댐퍼 owner 는 OWNER_INSTANCE_TYPE='MODEL'(TB_DUCT/OBSTACLE 에 없음).
- **유틸리티 라벨** = `"[UTILITY_GROUP] SOURCE_UTILITY"`. 라우팅은 유틸리티별 그룹핑.
- **종단객체(시각화)**: `TB_LATERAL_PIPE` + `TB_DUCT`(구 TB_DUCT_LATERAL 분리, AABB_*·UTILITY). **공간영역**: `TB_SPACE_INFO`(층, 건물 전체라 그룹 AABB 로 클리핑·와이어프레임). **학습자산**: 공식 `TB_ROUTE_DESIGN_GROUP`(번들)·`TB_ROUTE_SEGMENT_TEMPLATE`(스텁 면, SEGMENT_ROLE A_EQUIP_STUB/C_DUCT_ENTRY).
- 단위 **mm**. 격자=그룹 AABB 3축 클램프(셀 폭발 방지). 검증값(그룹1=CLEAN/WTNHJ02): 장애물 1177(damper 제외)→클리핑 1139, 작업 151, 종단 20.
- **Npgsql 함정**: 연결당 reader 1개(MARS 미지원) — 한 conn 에서 reader 중첩 금지(블록스코프로 닫기). PatternStore/BundleStore 에서 반복 재발.

**How to apply:** 뷰어 `Model/ObstacleDbLoader.cs`(C#) 또는 Python `routing3d_py.{scene,route_db}`(`list_projects`/`list_groups`·`load_scene(project_id)`·`load_existing_pipes(xy_bbox)`). 전수 분석 엑셀 = `out/_gen_route_analysis_xlsx.py`. 관련: [[reference-obstacle-db]], [[project-routing3d]]
