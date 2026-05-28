// 라우팅 작업 타입 (RouteTask) — Routing3D C++ 엔진 (Phase 3)
// =============================================================================
// [이 파일이 하는 일]
//   배관 1건의 라우팅 작업(시작/끝 월드좌표 + 유틸리티/이름 메타)을 정의한다.
//   multi_route(순차 라우팅)와 scene_io(scene.txt 직렬화)가 공유하는 입력 타입이다.
//   Python 레퍼런스 routing3d_py/scene.py 의 RouteTask 와 1:1 대응.
//
//   선택 문자열 필드는 std::optional 로 표현한다. **None(미설정)과 빈 문자열("")을
//   구분**해야 scene.txt 왕복이 무손실(계약 F3)이기 때문이다(\N=None vs ""=빈문자열).
// =============================================================================
#pragma once

#include <optional>
#include <string>

#include "routing3d/geometry.hpp"

namespace routing3d {

// 라우팅 작업 1건. 경로 = start_mm → end_mm (월드좌표 mm).
struct RouteTask {
    Vec3 start_mm;
    Vec3 end_mm;
    std::optional<std::string> utility;            // 유틸리티 이름 (예: "NW").
    std::optional<std::string> utility_group;      // 유틸리티 그룹 (예: "Water").
    std::optional<std::string> start_name;         // 시작 PoC 이름.
    std::optional<std::string> end_name;           // 끝 PoC 이름.
    std::optional<std::string> end_instance_guid;  // 끝 객체 GUID.

    // 유틸리티 라벨 = "[그룹] 유틸" (예: "[Water] NW"). priority="utility" 그룹핑/정렬 키.
    // Python `group or '?'` 의미를 따른다: None **또는 빈 문자열**이면 '?'.
    std::string utility_label() const {
        auto pick = [](const std::optional<std::string>& s) -> std::string {
            return (s && !s->empty()) ? *s : std::string("?");
        };
        return "[" + pick(utility_group) + "] " + pick(utility);
    }
};

}  // namespace routing3d
