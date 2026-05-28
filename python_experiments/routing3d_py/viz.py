"""점유맵 3D 시각화 (PyVista) — Phase 1
================================================================================

[실행 명령어]  (모두 python_experiments/ 디렉토리에서)
  # ① DB 영역을 OST_TYPE별 색상으로 렌더 → 스크린샷 PNG 저장 (창 없이)
  ..\\.venv\\Scripts\\python.exe -m routing3d_py.viz ^
      --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 ^
      --screenshot out/occupancy.png

  # ② 같은 장면을 인터랙티브 창으로 보기 (마우스로 회전/줌)
  ..\\.venv\\Scripts\\python.exe -m routing3d_py.viz ^
      --region 195000 8000 14000 205000 12000 16000 --show

  # ③ 브라우저에서 볼 수 있는 인터랙티브 HTML 로 내보내기
  ..\\.venv\\Scripts\\python.exe -m routing3d_py.viz ^
      --region 195000 8000 14000 205000 12000 16000 --html out/occupancy.html

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
OccupancyMap(어느 백엔드든)을 PyVista 로 3차원 복셀로 렌더한다. 점유된 셀만
'복셀 머지' 방식(VTK ImageData + threshold)으로 뽑아 메시로 그리므로, 수십만
셀도 가볍게 표시된다. OST_TYPE 별로 점유맵을 따로 만들어 색을 달리하면 바닥/기둥/
보 등을 한눈에 구분할 수 있다.

[전체 흐름도]
--------------------------------------------------------------------------------
  OccupancyMap.to_numpy()  →  (nx,ny,nz) bool 배열
        │
        ▼  occupancy_to_voxels()
  pv.ImageData(셀=복셀) + cell_data['blocked']  →  threshold(0.5)
        │   = 점유 셀만 남긴 복셀 메시(UnstructuredGrid)
        ▼  render_occupancy(layers=...)
  pv.Plotter 에 레이어(색/라벨/투명도)별 add_mesh
        │
        ├─ show=True       → 인터랙티브 창
        ├─ screenshot=path → PNG 저장(off-screen, 창 없이)
        └─ html=path       → 브라우저용 인터랙티브 HTML

[출력 방식 선택 가이드]
  - 빠른 결과 확인        → screenshot (PNG). 창이 필요 없어 어디서나 동작.
  - 직접 돌려보며 확인     → show (인터랙티브 창). 로컬 데스크톱에서.
  - 공유/원격 확인        → html (브라우저).
================================================================================
"""

from __future__ import annotations

import os
import sys

import numpy as np

from .occupancy import OccupancyMap

# OST_TYPE → 색상 매핑(시각화 기본값). 없는 타입은 PALETTE 로 순환 배정.
DEFAULT_TYPE_COLORS: dict[str, str] = {
    "OST_Floors": "lightgray",          # 바닥
    "OST_Columns": "steelblue",         # 기둥
    "OST_StructuralColumns": "royalblue",  # 구조 기둥
    "OST_BeamStartSegment": "orange",   # 보
    "OST_StructuralFraming": "gold",    # 구조 프레임
    "OST_Ceilings": "mediumseagreen",   # 천장
    "OST_Walls": "salmon",              # 벽
    "": "dimgray",                      # 타입 미상
}

# 색상이 지정되지 않은 레이어에 순환 배정할 팔레트.
PALETTE: tuple[str, ...] = (
    "steelblue", "orange", "mediumseagreen", "salmon", "gold",
    "mediumpurple", "lightgray", "teal", "crimson", "olive",
)


def occupancy_to_voxels(occ: OccupancyMap):
    """OccupancyMap 을 '점유 셀만 남긴' PyVista 복셀 메시로 변환한다.

    [알고리즘]
      1) occ.to_numpy() 로 (nx,ny,nz) bool 배열을 얻는다(백엔드 무관).
      2) 셀 크기·원점에 맞춘 pv.ImageData(복셀 격자)를 만든다.
         - 점(point) 차원 = (nx+1, ny+1, nz+1), 셀(cell) 개수 = nx*ny*nz.
      3) cell_data['blocked'] 에 점유 플래그(0/1)를 VTK 순서(x 가장 빠름)로 넣는다.
      4) threshold(0.5) 로 점유 셀(=1)만 추출 → 인접 복셀이 병합된 메시.

    매개변수:
        occ : OccupancyMap (Dense/Sparse/BitPacked 무관).
    반환값:
        pv.UnstructuredGrid — 점유 셀 복셀 메시(점유 0개면 비어 있음).
    """
    import pyvista as pv

    arr = occ.to_numpy()
    nx, ny, nz = occ.shape
    cell = occ.cell_mm
    grid = pv.ImageData(
        dimensions=(nx + 1, ny + 1, nz + 1),
        spacing=(cell, cell, cell),
        origin=tuple(float(v) for v in occ.origin),
    )
    # numpy (i=x,j=y,k=z) → VTK 셀 순서(x 가장 빠름)는 order='F' 평탄화와 일치.
    grid.cell_data["blocked"] = arr.flatten(order="F").astype(np.uint8)
    return grid.threshold(0.5, scalars="blocked")


def full_grid_edges(occ: OccupancyMap):
    """격자 '전체 셀'(빈 셀 포함)의 모든 모서리를 선(line) 메시로 반환한다.

    점유 여부와 무관하게 region 을 cell_mm 간격으로 분할한 복셀 격자 자체를
    와이어프레임으로 보여주기 위한 것. 점유셀(occupancy_to_voxels)과 겹쳐 그리면
    "전체 복셀 격자 + 그 안의 점유셀"을 함께 볼 수 있다.

    매개변수:
        occ : OccupancyMap. shape/origin/cell_mm 만 사용(점유 내용은 무관).
    반환값:
        pv.PolyData — 모든 셀 모서리를 담은 선 메시.

    주의:
        셀 수가 매우 크면(예: 수십만) 선이 폭증해 무겁고 지저분하다.
        '전체 복셀'을 보고 싶을 땐 작은 region(예: 수천 셀)에서 사용할 것.
    """
    import pyvista as pv

    nx, ny, nz = occ.shape
    cell = occ.cell_mm
    grid = pv.ImageData(
        dimensions=(nx + 1, ny + 1, nz + 1),
        spacing=(cell, cell, cell),
        origin=tuple(float(v) for v in occ.origin),
    )
    # 모든 셀의 고유 모서리만 추출 → 격자 라인.
    return grid.extract_all_edges()


def _resolve_color(label: str, index: int, colors: dict[str, str] | None) -> str:
    """레이어 라벨에 대한 색을 결정한다(사용자 지정 > 타입 기본값 > 팔레트 순환)."""
    if colors and label in colors:
        return colors[label]
    if label in DEFAULT_TYPE_COLORS:
        return DEFAULT_TYPE_COLORS[label]
    return PALETTE[index % len(PALETTE)]


def render_occupancy(
    layers: OccupancyMap | dict[str, OccupancyMap],
    *,
    colors: dict[str, str] | None = None,
    opacity: float = 1.0,
    show_edges: bool = False,
    edge_color: str = "gray",
    all_voxels: bool = False,
    grid_color: str = "lightgray",
    grid_opacity: float = 0.25,
    path: list[tuple[int, int, int]] | None = None,
    path_color: str = "red",
    visited: list[tuple[int, int, int]] | None = None,
    visited_color: str = "khaki",
    visited_opacity: float = 0.12,
    polylines: list[tuple[list[tuple[float, float, float]], str, str]] | None = None,
    line_radius: float | None = None,
    show: bool = False,
    screenshot: str | None = None,
    html: str | None = None,
    background: str = "white",
    show_grid: bool = True,
    window_size: tuple[int, int] = (1280, 960),
    title: str = "Routing3D Occupancy",
):
    """점유맵 레이어를 PyVista 로 렌더하고 화면/PNG/HTML 로 출력한다.

    매개변수:
        layers     : 단일 OccupancyMap 또는 {라벨: OccupancyMap} 딕셔너리.
                     딕셔너리면 라벨별로 색을 달리해 그린다(예: OST_TYPE 별).
        colors     : {라벨: 색이름} 사용자 지정 색(선택).
        opacity    : 점유셀 복셀 불투명도 0~1 (기본 1.0). 내부를 보려면 낮춘다.
        show_edges : True 면 점유 복셀마다 셀 윤곽선을 그려 개별 셀이 보이게 한다.
        edge_color : 점유 복셀 윤곽선 색(기본 gray).
        all_voxels : True 면 region 전체의 '모든 복셀 격자'를 와이어프레임으로 함께
                     그린다(빈 셀 포함). 점유셀과 겹쳐 보여 전체 격자 대비 점유 위치를
                     파악할 수 있다. (셀 수가 크면 무거우니 작은 region 권장)
        grid_color : 전체 격자 와이어프레임 색(기본 lightgray).
        grid_opacity: 전체 격자 와이어프레임 불투명도(기본 0.25).
        path       : A* 경로 셀 리스트(선택). 주어지면 경로를 튜브로, 시작=초록·
                     목표=빨강 구로 표시한다. 셀→월드 변환은 첫 레이어 맵 기준.
        path_color : 경로 튜브 색(기본 red).
        visited    : A* 가 확장한 방문 셀 리스트(선택, AStarResult.visited). 주어지면
                     탐색이 훑은 영역을 반투명 복셀 레이어로 함께 그린다(탐색 범위 가시화).
                     첫 레이어의 shape/origin/cell_mm 기준 임시 점유맵으로 머지 렌더.
        visited_color  : 방문 레이어 색(기본 khaki).
        visited_opacity: 방문 레이어 불투명도(기본 0.12, 경로·점유가 비치도록 낮게).
        polylines  : 월드좌표(mm) 다중 선분 리스트. 각 원소 (점목록, 색, 라벨).
                     유틸리티별 경로(또는 start→end 직선)를 색으로 묶어 그릴 때 사용.
                     같은 라벨은 범례에 한 번만 표시.
        line_radius: polylines 튜브 반경(mm). None 이면 첫 레이어 cell_mm*0.3.
        show       : True 면 인터랙티브 창을 띄운다(로컬 데스크톱).
        screenshot : 경로가 주어지면 PNG 스크린샷을 저장한다(off-screen, 창 불필요).
        html       : 경로가 주어지면 브라우저용 인터랙티브 HTML 로 내보낸다.
        background : 배경색(기본 white).
        show_grid  : True 면 mm 눈금 축을 표시.
        window_size: 렌더 해상도(px).
        title      : 창/스크린샷 제목.
    반환값:
        사용한 pv.Plotter (추가 조작용).

    지역 변수:
        layer_map : {라벨: OccupancyMap} 로 정규화한 입력.
        off_screen: 창을 띄우지 않는 경우 True(스크린샷/HTML 전용).
        n_total   : 그린 점유 셀 총 개수(로그용).
    """
    import pyvista as pv

    layer_map: dict[str, OccupancyMap]
    if isinstance(layers, OccupancyMap):
        layer_map = {"occupancy": layers}
    else:
        layer_map = layers

    off_screen = not show
    plotter = pv.Plotter(off_screen=off_screen, window_size=list(window_size))
    plotter.set_background(background)

    # ① 전체 복셀 격자(빈 셀 포함)를 와이어프레임으로 먼저 그린다.
    if all_voxels and layer_map:
        any_occ = next(iter(layer_map.values()))
        edges = full_grid_edges(any_occ)
        plotter.add_mesh(edges, color=grid_color, opacity=grid_opacity,
                         line_width=1, label="voxel grid")

    # ② 점유 셀을 레이어(색)별로 그린다.
    n_total = 0
    for idx, (label, occ) in enumerate(layer_map.items()):
        voxels = occupancy_to_voxels(occ)
        if voxels.n_cells == 0:
            continue
        n_total += occ.count_blocked()
        plotter.add_mesh(
            voxels,
            color=_resolve_color(label, idx, colors),
            opacity=opacity,
            show_edges=show_edges,
            edge_color=edge_color,
            label=label,
        )

    # ②.5 방문(visited) 셀: 탐색이 훑은 영역을 반투명 복셀 레이어로(첫 레이어 격자 기준).
    if visited and layer_map:
        from .occupancy import DenseOccupancyMap

        ref = next(iter(layer_map.values()))
        vocc = DenseOccupancyMap(ref.shape, tuple(ref.origin), ref.cell_mm)
        for c in visited:
            vocc.block_cell(c)
        vmesh = occupancy_to_voxels(vocc)
        if vmesh.n_cells:
            plotter.add_mesh(vmesh, color=visited_color, opacity=visited_opacity,
                             label="visited")

    # ③ A* 경로(있으면): 셀 중심을 잇는 튜브 + 시작/목표 마커.
    if path and layer_map:
        ref = next(iter(layer_map.values()))
        pts = np.array([ref.to_world(c) for c in path], dtype=float)
        r = ref.cell_mm * 0.35
        if len(pts) >= 2:
            tube = pv.lines_from_points(pts).tube(radius=r)
            plotter.add_mesh(tube, color=path_color, label="path")
        plotter.add_mesh(pv.Sphere(radius=ref.cell_mm * 0.7, center=pts[0]),
                         color="lime", label="start")
        plotter.add_mesh(pv.Sphere(radius=ref.cell_mm * 0.7, center=pts[-1]),
                         color="red", label="goal")

    # ④ 유틸리티별 다중 선분(월드 mm): 같은 라벨은 범례에 1회만.
    if polylines:
        ref = next(iter(layer_map.values())) if layer_map else None
        radius = line_radius if line_radius is not None else (
            ref.cell_mm * 0.3 if ref is not None else 30.0)
        seen_labels: set[str] = set()
        for pts_mm, color, label in polylines:
            if len(pts_mm) < 2:
                continue
            arr = np.asarray(pts_mm, dtype=float)
            tube = pv.lines_from_points(arr).tube(radius=radius)
            lbl = None if label in seen_labels else label
            seen_labels.add(label)
            plotter.add_mesh(tube, color=color, label=lbl)

    if len(layer_map) > 1 or path or polylines or visited:
        plotter.add_legend(bcolor="white", border=True)
    if show_grid:
        # 라벨 개수를 줄여 큰 mm 좌표값이 원점 부근에서 겹치는 것을 방지.
        plotter.show_grid(
            xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
            n_xlabels=4, n_ylabels=4, n_zlabels=3, fmt="%.0f",
        )
    plotter.add_axes()
    plotter.add_text(f"{title}  |  voxels={n_total}", font_size=10)

    # 보기 좋은 등각(isometric) 시점.
    plotter.view_isometric()

    if screenshot:
        os.makedirs(os.path.dirname(os.path.abspath(screenshot)), exist_ok=True)
        plotter.screenshot(screenshot)
        print(f"[스크린샷 저장] {screenshot}  (voxels={n_total})")
    if html:
        os.makedirs(os.path.dirname(os.path.abspath(html)), exist_ok=True)
        plotter.export_html(html)
        print(f"[HTML 저장] {html}  (voxels={n_total})")
    if show:
        plotter.show(title=title)

    return plotter


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    """커맨드라인 진입점. DB 영역을 OST_TYPE별로 렌더한다. 상단 [실행 명령어] 참고."""
    import argparse

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="점유맵 3D 시각화 (PyVista)")
    parser.add_argument("--region", nargs=6, type=float, required=True,
                        metavar=("MINX", "MINY", "MINZ", "MAXX", "MAXY", "MAXZ"),
                        help="관심 영역 mm (min xyz, max xyz)")
    parser.add_argument("--cell-mm", type=float, default=50.0, help="셀 크기 mm (기본 50)")
    parser.add_argument("--types", nargs="+", default=None, help="가져올 OST_TYPE 목록")
    parser.add_argument("--no-by-type", action="store_true",
                        help="타입 구분 없이 단일 색으로 렌더")
    parser.add_argument("--opacity", type=float, default=1.0, help="점유셀 복셀 불투명도 0~1")
    parser.add_argument("--edges", action="store_true",
                        help="점유 복셀마다 셀 윤곽선 표시(개별 셀이 보임)")
    parser.add_argument("--all-voxels", action="store_true",
                        help="region 전체 복셀 격자(빈 셀 포함)를 와이어프레임으로 함께 표시")
    parser.add_argument("--show", action="store_true", help="인터랙티브 창 표시")
    parser.add_argument("--screenshot", default=None, help="PNG 스크린샷 경로")
    parser.add_argument("--html", default=None, help="인터랙티브 HTML 경로")
    parser.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = parser.parse_args(argv)

    # 출력 인자가 하나도 없으면 기본 스크린샷 경로 사용(창 없이 결과 확인).
    if not (args.show or args.screenshot or args.html):
        args.screenshot = "out/occupancy.png"

    from .obstacle_db import (
        PgConnConfig,
        build_occupancy,
        group_by_type,
        load_obstacles,
    )

    region = (tuple(args.region[0:3]), tuple(args.region[3:6]))
    overrides = {"dbname": args.dbname} if args.dbname else {}
    config = PgConnConfig.from_env(**overrides)

    obstacles = load_obstacles(config, ost_types=args.types, region=region)
    print(f"로드된 장애물: {len(obstacles):,}건")
    if not obstacles:
        print("조건에 맞는 장애물이 없습니다. 종료합니다.")
        return 0

    if args.no_by_type:
        layers: OccupancyMap | dict[str, OccupancyMap] = build_occupancy(
            obstacles, cell_mm=args.cell_mm, region=region
        ).occupancy
    else:
        # OST_TYPE 별로 각각 점유맵을 구성(같은 region 으로 정렬 일치).
        groups = group_by_type(obstacles)
        layers = {}
        for ost_type, group in sorted(groups.items()):
            res = build_occupancy(group, cell_mm=args.cell_mm, region=region)
            layers[ost_type or "(미상)"] = res.occupancy
            print(f"  {ost_type or '(미상)':24s}: 점유셀 {res.occupancy.count_blocked():,}")

    render_occupancy(
        layers,
        opacity=args.opacity,
        show_edges=args.edges,
        all_voxels=args.all_voxels,
        show=args.show,
        screenshot=args.screenshot,
        html=args.html,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
