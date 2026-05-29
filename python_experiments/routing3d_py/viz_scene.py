"""scene.txt 결과 3D 가시화 (C++ 엔진 산출물 시각화) — Phase 3 보조
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  # ① C++ CLI 가 만든 결과 scene.txt 를 3D 로 렌더 → PNG 저장(창 없이)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.viz_scene ^
      --in out/demo.scene.txt --screenshot out/demo.png

  # ② 브라우저로 볼 수 있는 인터랙티브 HTML 로 내보내기
  .\\.venv\\Scripts\\python.exe -m routing3d_py.viz_scene ^
      --in out/routed_multi.scene.txt --html out/routed_multi.html

  # ③ 인터랙티브 창으로 직접 회전/줌 (로컬 데스크톱)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.viz_scene --in out/demo.scene.txt --show

  # ④ 특정 유틸리티만 + 방문(visited) 레이어까지 함께
  .\\.venv\\Scripts\\python.exe -m routing3d_py.viz_scene ^
      --in out/demo.scene.txt --utility "[Gas] PA" --visited --screenshot out/pa.png

  # 전체 파이프라인 예: C++ 엔진으로 라우팅 → 결과를 그대로 가시화
  .\\run.ps1 route --in scene.txt --out out/routed.scene.txt --mode multi
  .\\.venv\\Scripts\\python.exe -m routing3d_py.viz_scene --in out/routed.scene.txt --screenshot out/routed.png

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
Phase 3 C++ 엔진(routing3d_cli)이 출력한 결과 scene.txt 를 읽어, 기존 Python 결과
가시화 모듈(routing3d_py.viz)을 그대로 활용해 3차원으로 그린다. 즉 "엔진은 C++, 보기는
기존 Python 뷰어"로 이어 붙여 지금까지 개발한 라우팅 결과를 눈으로 확인한다.

scene.txt 는 Python(scene_io)과 C++(scene_io.hpp)이 공유하는 v1 포맷이므로, C++ CLI 의
산출물과 Python 의 산출물을 동일한 코드로 렌더할 수 있다(포맷 계약 F2~F4).

[전체 흐름도]
--------------------------------------------------------------------------------
  scene.txt (C++ routing3d_cli 산출물 / 또는 Python 산출물)
        │  read_scene()
        ▼
  SceneDoc(격자·장애물·작업·결과)
        │  occupancy_from_doc()                 scene_doc_polylines()
        ▼                                              │ (작업별 성공 경로 셀→월드, 유틸리티 색)
  점유맵(obstacles 레이어)  ───────────────┐          ▼
        │                                  └──▶  polylines: [(점목록 mm, 색, 유틸 라벨)]
        ▼  viz.render_occupancy({"obstacles": occ}, opacity=0.12, polylines=...)
  PyVista 3D
        ├─ show=True       → 인터랙티브 창
        ├─ screenshot=path → PNG(off-screen, 창 불필요)
        └─ html=path       → 브라우저용 인터랙티브 HTML

[색/레이어 규약 — 기존 모듈과 동일]
  - 장애물: 반투명(opacity 0.12) 복셀(내부 경로가 비치도록).
  - 경로  : 유틸리티 라벨('[그룹] 유틸')별 색(utility_colors, 결정적 배정), 튜브로.
  - 방문  : (선택) 탐색이 훑은 셀을 반투명 복셀 레이어로(모든 작업 합집합).
================================================================================
"""

from __future__ import annotations

import os
import sys

from .occupancy import OccupancyMap
from .scene import utility_colors
from .scene_io import SceneDoc, occupancy_from_doc, read_scene

Vec3 = tuple[float, float, float]


def scene_doc_polylines(
    doc: SceneDoc,
    occ: OccupancyMap,
    *,
    colors: dict[str, str] | None = None,
    utility: str | None = None,
) -> list[tuple[list[Vec3], str, str]]:
    """SceneDoc 의 결과 경로를 viz.render_occupancy(polylines=...) 용 선분 목록으로 변환한다.

    각 작업(task)과 평행한 결과(result)에서 '성공 + 경로 있음'인 것만 골라, 경로 셀을
    월드좌표(mm)로 바꿔 유틸리티 라벨별 색을 입힌다(기존 scene.scene_polylines 와 동일 규약).

    매개변수:
        doc     : read_scene 으로 읽은 SceneDoc(results 는 tasks 와 평행).
        occ     : 셀→월드 변환에 쓸 점유맵(occupancy_from_doc 결과).
        colors  : 유틸리티 라벨→색 매핑(선택). None 이면 작업들의 라벨로 자동 배정.
        utility : 이 유틸리티 라벨만 그릴 때 지정(예: "[Gas] PA"). None 이면 전체.
    반환값:
        list[(점목록 mm, 색, 유틸리티 라벨)].

    지역 변수:
        labels : 작업들에서 모은 유틸리티 라벨(색 배정 기준).
        lines  : 누적 선분 목록.
    """
    labels = sorted({t.utility_label for t in doc.tasks})
    colors = colors or utility_colors(labels)
    lines: list[tuple[list[Vec3], str, str]] = []
    for task, res in zip(doc.tasks, doc.results):
        if res is None or not res.success or not res.path:
            continue
        label = task.utility_label
        if utility is not None and label != utility:
            continue
        pts = [occ.to_world(c) for c in res.path]
        lines.append((pts, colors.get(label, "gray"), label))
    return lines


def merged_visited(doc: SceneDoc, *, utility: str | None = None) -> list[tuple[int, int, int]]:
    """모든 작업의 방문(visited) 셀을 중복 없이 합친다(탐색 범위 가시화용).

    매개변수:
        doc     : SceneDoc.
        utility : 이 유틸리티 라벨의 작업만 합칠 때 지정. None 이면 전체.
    반환값:
        방문 셀 리스트(입력 등장 순서, 중복 제거).
    """
    seen: set[tuple[int, int, int]] = set()
    out: list[tuple[int, int, int]] = []
    for task, res in zip(doc.tasks, doc.results):
        if res is None or not res.visited:
            continue
        if utility is not None and task.utility_label != utility:
            continue
        for c in res.visited:
            ck = (int(c[0]), int(c[1]), int(c[2]))
            if ck not in seen:
                seen.add(ck)
                out.append(ck)
    return out


def _print_summary(doc: SceneDoc, utility: str | None) -> None:
    """씬 요약 + 유틸리티별 성공/전체 출력(콘솔)."""
    print(doc.summary())
    # 유틸리티별 성공/전체 집계.
    per: dict[str, list[bool]] = {}
    for task, res in zip(doc.tasks, doc.results):
        ok = bool(res is not None and res.success)
        per.setdefault(task.utility_label, []).append(ok)
    print("유틸리티별 성공/전체:")
    for label, oks in sorted(per.items(), key=lambda x: -len(x[1])):
        mark = "" if (utility is None or label == utility) else "  (숨김)"
        print(f"  {label}: {sum(oks)}/{len(oks)}{mark}")


def render_scene_file(
    path: str,
    *,
    obstacle_opacity: float = 0.12,
    show_visited: bool = False,
    utility: str | None = None,
    show: bool = False,
    screenshot: str | None = None,
    html: str | None = None,
    title: str | None = None,
):
    """결과 scene.txt 한 개를 읽어 점유맵 + 경로(+선택 방문)를 3D 로 렌더한다.

    매개변수:
        path             : 읽을 scene.txt 경로(C++ CLI 또는 Python 산출물).
        obstacle_opacity : 장애물 복셀 불투명도(기본 0.12, 경로가 비치도록 낮게).
        show_visited     : True 면 모든 작업의 방문 셀을 반투명 레이어로 함께 그린다.
        utility          : 이 유틸리티 라벨만 경로/방문 표시(예: "[Gas] PA").
        show/screenshot/html : 출력 방식(인터랙티브 창 / PNG / HTML). viz 와 동일.
        title            : 렌더 제목(기본 파일명).
    반환값:
        읽은 SceneDoc.

    지역 변수:
        doc       : read_scene 결과.
        occ       : occupancy_from_doc 로 복원한 장애물 점유맵.
        polylines : 유틸리티별 경로 선분 목록.
        visited   : show_visited 일 때 합친 방문 셀(아니면 None).
    """
    doc = read_scene(path)
    occ = occupancy_from_doc(doc)
    _print_summary(doc, utility)

    polylines = scene_doc_polylines(doc, occ, utility=utility)
    visited = merged_visited(doc, utility=utility) if show_visited else None

    # 기존 결과 가시화 모듈(viz)을 그대로 사용 — 장애물 반투명 + 유틸리티별 경로 색.
    from .viz import render_occupancy

    render_occupancy(
        {"obstacles": occ},
        opacity=obstacle_opacity,
        polylines=polylines,
        visited=visited,
        show=show,
        screenshot=screenshot,
        html=html,
        title=title or os.path.basename(path),
    )
    return doc


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    """커맨드라인 진입점. 결과 scene.txt 를 3D 로 렌더한다. 상단 [실행 명령어] 참고."""
    import argparse

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="결과 scene.txt 3D 가시화 (C++ 엔진 산출물)")
    parser.add_argument("--in", dest="in_path", required=True, help="읽을 결과 scene.txt 경로")
    parser.add_argument("--screenshot", default=None, help="PNG 스크린샷 경로")
    parser.add_argument("--html", default=None, help="인터랙티브 HTML 경로")
    parser.add_argument("--show", action="store_true", help="인터랙티브 창 표시")
    parser.add_argument("--visited", action="store_true", help="방문(탐색) 셀 레이어 함께 표시")
    parser.add_argument("--utility", default=None, help="이 유틸리티 라벨만 (예: \"[Gas] PA\")")
    parser.add_argument("--obstacle-opacity", type=float, default=0.12,
                        help="장애물 복셀 불투명도 0~1 (기본 0.12)")
    args = parser.parse_args(argv)

    # 출력 인자가 하나도 없으면 입력 파일명 기준 기본 스크린샷 경로 사용(창 없이 확인).
    if not (args.show or args.screenshot or args.html):
        base = os.path.splitext(os.path.basename(args.in_path))[0]
        args.screenshot = os.path.join("out", base + ".png")

    render_scene_file(
        args.in_path,
        obstacle_opacity=args.obstacle_opacity,
        show_visited=args.visited,
        utility=args.utility,
        show=args.show,
        screenshot=args.screenshot,
        html=args.html,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
