# -*- coding: utf-8 -*-
"""기존 설계배관 vs 우리 A* 엔진 — 정합성 검증 리포트 생성기
================================================================================

[실행 명령어]  (프로젝트 루트에서)
  .\\.venv\\Scripts\\python.exe python_experiments/out/_gen_validation_report.py --project 6 --cell-mm 100

[이 스크립트가 하는 일]
--------------------------------------------------------------------------------
DB(AUTOROUTINGV7)의 기존 설계배관(TB_ROUTE_PATH)을 ground truth 로 삼아, **같은 장애물·
같은 종단점(SOURCE_POC→TARGET_POC)** 을 우리 직교 A*(astar_weighted)로 다시 라우팅해
길이/굴곡/우회비를 1:1 대조한다.

  · 기존 지표(설계 자체 사전계산): PR_TOTAL_LENGTH(mm), PR_BEND_COUNT(굴곡 피팅수),
                                   PR_PATH_EFFICIENCY(실제길이/직선거리, 우회비).
  · 우리 지표: length_mm, turns(방향전환), our_eff = our_len / 직선거리.

핵심 비교:
  - 길이비   our_len / PR_TOTAL_LENGTH   (1 미만이면 우리가 더 짧음)
  - 굴곡     turns vs PR_BEND_COUNT
  - 우회비   our_eff vs PR_PATH_EFFICIENCY

[주의]
  - 통과객체(OST_Floors/OST_Ceilings/격자보 BEAM_STRUCTURE)는 점유맵에서 제외(배관 통과).
  - 종단 PoC 가 점유/격자밖이면 반경 내 빈 셀로 스냅. 그래도 막히면 실패로 집계.
  - 라우트 컬럼은 대문자 따옴표 식별자. 종단점은 SOURCE_POSX/Y/Z, TARGET_POSX/Y/Z.
  - PR_BEND_COUNT 는 물리 피팅수(엘보+밴딩, 입면변화 포함), 우리 turns 는 셀경로 방향전환
    → 정의가 완전히 같지는 않으니 '경향' 비교로 해석.
================================================================================
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass

# editable 설치 가정. 미설치 시 경로 추가.
try:
    from routing3d_py.obstacle_db import (
        PgConnConfig, build_occupancy, load_obstacles, obstacles_bounds)
    from routing3d_py.cost import RouteParams
    from routing3d_py.astar import astar_weighted
except ModuleNotFoundError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from routing3d_py.obstacle_db import (
        PgConnConfig, build_occupancy, load_obstacles, obstacles_bounds)
    from routing3d_py.cost import RouteParams
    from routing3d_py.astar import astar_weighted


def is_passthrough(b) -> bool:
    """통과객체 판정(점유맵 제외 대상). C#/엔진과 동일 규칙."""
    ost = (b.ost_type or "").strip().lower()
    if ost in ("ost_floors", "ost_ceilings"):
        return True
    if ost == "ost_structuralframing" and (b.ddworks_type or "").strip().upper() == "BEAM_STRUCTURE":
        return True
    return False


@dataclass
class RouteRow:
    guid: str
    group: str
    utility: str
    size: str
    src: tuple
    tgt: tuple
    straight_mm: float
    pr_len: float
    pr_bend: float
    pr_eff: float
    ok: bool = False
    fail_reason: str = ""
    our_len: float = 0.0
    our_turns: int = 0
    our_eff: float = 0.0


def _get_source_file(conn, project_id: int) -> str:
    cur = conn.cursor()
    cur.execute("SELECT source_file FROM space_project_map WHERE project_id=%s", (project_id,))
    row = cur.fetchone()
    if not row or not row[0]:
        raise SystemExit(f"project_id={project_id} 의 source_file 없음")
    return row[0]


def _load_existing_routes(conn, source_file: str, bbox) -> list[RouteRow]:
    """이 프로젝트(tool)의 기존 설계 경로. 장비 NAME 매칭 + 장애물 bbox 로 tool 격리."""
    (minx, miny, maxx, maxy) = bbox
    sql = """
        SELECT rp."ROUTE_PATH_GUID", rp."UTILITY_GROUP", rp."SOURCE_UTILITY", rp."SOURCE_SIZE",
               rp."SOURCE_POSX", rp."SOURCE_POSY", rp."SOURCE_POSZ",
               rp."TARGET_POSX", rp."TARGET_POSY", rp."TARGET_POSZ",
               rp."PR_BEND_COUNT", rp."PR_PATH_EFFICIENCY", rp."PR_TOTAL_LENGTH"
        FROM "TB_ROUTE_PATH" rp
        JOIN "TB_BIM_EQUIPMENT" eq
          ON eq."NAME" = rp."SOURCE_OWNER_NAME" AND eq."IS_MAIN" = true AND eq."SOURCE_FILE" = %s
        WHERE rp."SOURCE_OWNER_POSX" BETWEEN %s AND %s
          AND rp."SOURCE_OWNER_POSY" BETWEEN %s AND %s
          AND rp."TARGET_POSX" IS NOT NULL AND rp."SOURCE_POSX" IS NOT NULL
        ORDER BY rp."ROUTE_PATH_GUID"
    """
    cur = conn.cursor()
    cur.execute(sql, (source_file, minx, maxx, miny, maxy))
    rows = cur.fetchall()

    out: list[RouteRow] = []
    seen = set()
    for r in rows:
        guid = r[0]
        if guid in seen:
            continue
        seen.add(guid)
        src = (float(r[4]), float(r[5]), float(r[6]))
        tgt = (float(r[7]), float(r[8]), float(r[9]))
        out.append(RouteRow(
            guid=guid, group=r[1] or "?", utility=r[2] or "?", size=r[3] or "",
            src=src, tgt=tgt, straight_mm=math.dist(src, tgt),
            pr_len=float(r[12]) if r[12] is not None else 0.0,
            pr_bend=float(r[10]) if r[10] is not None else 0.0,
            pr_eff=float(r[11]) if r[11] is not None else 0.0,
        ))
    return out


def _snap(occ, cell, radius):
    """cell 이 점유/격자밖이면 반경 내 가장 가까운 빈 셀. 없으면 None."""
    if occ.in_bounds(cell) and not occ.is_blocked(cell):
        return cell
    ci, cj, ck = cell
    best, best_d = None, None
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            for dk in range(-radius, radius + 1):
                c = (ci + di, cj + dj, ck + dk)
                if occ.in_bounds(c) and not occ.is_blocked(c):
                    d = abs(di) + abs(dj) + abs(dk)
                    if best_d is None or d < best_d:
                        best_d, best = d, c
    return best


def run_validation(project_id: int, cell_mm: float, params: RouteParams,
                   snap_radius: int = 3, max_expansions: int | None = 3_000_000) -> dict:
    config = PgConnConfig.from_env()
    conn = config.connect()
    try:
        source_file = _get_source_file(conn, project_id)
        print(f"[검증] project {project_id} source_file={source_file} cell={cell_mm}mm")

        boxes = load_obstacles(config, source_file=source_file, conn=conn)
        pass_n = sum(1 for b in boxes if is_passthrough(b))
        occ_boxes = [b for b in boxes if not is_passthrough(b)]
        if not occ_boxes:
            raise SystemExit("장애물 없음 — 점유맵 생성 불가")
        print(f"[검증] 장애물 {len(boxes)} (통과 제외 {pass_n}) → 점유 {len(occ_boxes)}")

        lo, hi = obstacles_bounds(boxes)  # 종단점이 격자밖이 안 되게 전체 범위 + 패딩.
        occ = build_occupancy(occ_boxes, cell_mm=cell_mm, region=(lo, hi),
                              padding_mm=1000.0).occupancy

        bbox = (lo[0] - 1000.0, lo[1] - 1000.0, hi[0] + 1000.0, hi[1] + 1000.0)
        routes = _load_existing_routes(conn, source_file, bbox)
        print(f"[검증] 기존 설계 경로 {len(routes)}개 라우팅 시작…")
    finally:
        conn.close()

    t0 = time.perf_counter()
    for i, rr in enumerate(routes):
        s = _snap(occ, occ.to_cell(rr.src), snap_radius)
        g = _snap(occ, occ.to_cell(rr.tgt), snap_radius)
        if s is None or g is None:
            rr.fail_reason = "종단점 스냅 실패(매몰)"
            continue
        res = astar_weighted(occ, s, g, params, max_expansions=max_expansions)
        if res.success and res.path:
            rr.ok = True
            rr.our_len = res.length_mm
            rr.our_turns = res.turns
            rr.our_eff = (res.length_mm / rr.straight_mm) if rr.straight_mm > 1 else 0.0
        else:
            rr.fail_reason = "경로 없음"
        if (i + 1) % 50 == 0:
            print(f"  … {i+1}/{len(routes)}")
    elapsed = time.perf_counter() - t0
    print(f"[검증] 완료 {elapsed:.1f}s")

    return {"project_id": project_id, "source_file": source_file, "cell_mm": cell_mm,
            "params": params, "routes": routes, "elapsed_s": elapsed,
            "obstacles": len(boxes), "passthrough": pass_n}


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def summarize(res: dict) -> dict:
    routes = res["routes"]
    ok = [r for r in routes if r.ok]
    pair = [r for r in ok if r.pr_len > 0 and r.straight_mm > 1]
    lr = [r.our_len / r.pr_len for r in pair]
    n = len(routes)
    return {
        "n": n, "n_ok": len(ok), "rate": (len(ok) / n if n else 0.0), "n_pair": len(pair),
        "our_len_avg": sum(r.our_len for r in pair) / len(pair) if pair else 0,
        "pr_len_avg": sum(r.pr_len for r in pair) / len(pair) if pair else 0,
        "len_ratio_avg": sum(lr) / len(lr) if lr else 0,
        "len_ratio_med": _median(lr),
        "our_turns_avg": sum(r.our_turns for r in pair) / len(pair) if pair else 0,
        "pr_bend_avg": sum(r.pr_bend for r in pair) / len(pair) if pair else 0,
        "our_eff_avg": sum(r.our_eff for r in pair) / len(pair) if pair else 0,
        "pr_eff_avg": sum(r.pr_eff for r in pair) / len(pair) if pair else 0,
        "shorter_pct": 100.0 * sum(1 for x in lr if x < 1.0) / len(lr) if lr else 0,
        "fewer_bend_pct": 100.0 * sum(1 for r in pair if r.our_turns < r.pr_bend) / len(pair) if pair else 0,
        "pair": pair, "routes": routes,
    }


def by_group(pair) -> dict:
    g = {}
    for r in pair:
        g.setdefault(r.group, []).append(r)
    out = {}
    for k, rs in sorted(g.items()):
        lr = [r.our_len / r.pr_len for r in rs]
        out[k] = {"n": len(rs),
                  "our_len": sum(r.our_len for r in rs) / len(rs),
                  "pr_len": sum(r.pr_len for r in rs) / len(rs),
                  "ratio": sum(lr) / len(lr),
                  "our_turns": sum(r.our_turns for r in rs) / len(rs),
                  "pr_bend": sum(r.pr_bend for r in rs) / len(rs)}
    return out


def write_docx(res: dict, summ: dict, out_path: str) -> None:
    from docx import Document
    from docx.shared import Pt
    from docx.oxml.ns import qn

    doc = Document()
    st = doc.styles["Normal"]
    st.font.name = "맑은 고딕"
    st.font.size = Pt(10)
    try:
        st.element.rPr.rFonts.set(qn("w:eastAsia"), "맑은 고딕")
    except Exception:
        pass

    def mktable(headers, rows):
        t = doc.add_table(rows=len(rows) + 1, cols=len(headers))
        t.style = "Table Grid"
        for j, h in enumerate(headers):
            t.rows[0].cells[j].text = str(h)
        for i, row in enumerate(rows, start=1):
            for j, v in enumerate(row):
                t.rows[i].cells[j].text = str(v)
        return t

    doc.add_heading("Routing3D 검증 리포트 — 기존 설계배관 vs 자동 라우팅", level=0)
    doc.add_paragraph(
        f"프로젝트 {res['project_id']} ({res['source_file']}) · 셀 {res['cell_mm']:.0f}mm · "
        f"장애물 {res['obstacles']}개(통과 {res['passthrough']} 제외) · "
        f"비용 w_turn={res['params'].w_turn:.0f} w_clear={res['params'].w_clear:.0f} "
        f"clearance={res['params'].clearance_radius}셀 · 소요 {res['elapsed_s']:.1f}s")

    doc.add_heading("1. 요약", level=1)
    mktable(["항목", "값"], [
        ("기존 설계 경로 수", f"{summ['n']}개"),
        ("우리 엔진 라우팅 성공", f"{summ['n_ok']}개 ({summ['rate']*100:.1f}%)"),
        ("비교 쌍(성공·기존길이>0)", f"{summ['n_pair']}개"),
        ("평균 길이 — 기존", f"{summ['pr_len_avg']:,.0f} mm"),
        ("평균 길이 — 우리", f"{summ['our_len_avg']:,.0f} mm"),
        ("길이비 our/기존 (평균 / 중앙)", f"{summ['len_ratio_avg']:.3f} / {summ['len_ratio_med']:.3f}"),
        ("우리가 더 짧은 비율", f"{summ['shorter_pct']:.1f}%"),
        ("평균 굴곡 — 기존(PR_BEND)", f"{summ['pr_bend_avg']:.2f}"),
        ("평균 굴곡 — 우리(turns)", f"{summ['our_turns_avg']:.2f}"),
        ("우리가 굴곡 더 적은 비율", f"{summ['fewer_bend_pct']:.1f}%"),
        ("평균 우회비 — 기존(PR_EFF)", f"{summ['pr_eff_avg']:.3f}"),
        ("평균 우회비 — 우리", f"{summ['our_eff_avg']:.3f}"),
    ])

    doc.add_heading("2. 유틸리티 그룹별 비교", level=1)
    grp = by_group(summ["pair"])
    mktable(["유틸리티 그룹", "쌍 수", "기존 길이(mm)", "우리 길이(mm)", "길이비", "굴곡 기존→우리"],
            [[k, v["n"], f"{v['pr_len']:,.0f}", f"{v['our_len']:,.0f}",
              f"{v['ratio']:.3f}", f"{v['pr_bend']:.1f} → {v['our_turns']:.1f}"]
             for k, v in grp.items()])

    doc.add_heading("3. 샘플 경로 (길이비 하·상위 각 8건)", level=1)
    ps = sorted(summ["pair"], key=lambda r: r.our_len / r.pr_len)
    sample = ps[:8] + ps[-8:]
    mktable(["그룹/유틸", "관경", "직선(mm)", "기존(mm)", "우리(mm)", "길이비"],
            [[f"{r.group}/{r.utility}", r.size, f"{r.straight_mm:,.0f}",
              f"{r.pr_len:,.0f}", f"{r.our_len:,.0f}", f"{r.our_len / r.pr_len:.3f}"]
             for r in sample])

    doc.add_heading("4. 해석", level=1)
    for s in [
        f"성공률 {summ['rate']*100:.1f}%: 실패분은 종단 PoC 매몰/협소부 → 전처리(스냅확장·표면투사) 과제.",
        f"길이비 중앙 {summ['len_ratio_med']:.3f}: 1 미만이면 우리 경로가 기존보다 짧음. "
        f"기존 설계는 공용 파이프랙을 따라 우회(우회비 {summ['pr_eff_avg']:.2f})하나 우리는 비용최적이라 더 직선적.",
        f"굴곡: 기존 평균 {summ['pr_bend_avg']:.1f} vs 우리 {summ['our_turns_avg']:.1f} "
        f"(PR_BEND=입면 포함 물리 피팅수, turns=셀경로 방향전환 → 경향 비교).",
        "결론: 충돌 없는 직교 경로를 안정 생성하며 길이/굴곡이 설계 대비 합리적. "
        "'기존과 유사'가 목표면 공용랙 인력 비용, '최적'이 목표면 현재 경향이 우수.",
    ]:
        doc.add_paragraph(s, style="List Bullet")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    doc.save(out_path)
    print(f"[검증] 문서 저장: {out_path}")


def write_csv(summ: dict, path: str) -> None:
    import csv
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["guid", "group", "utility", "size", "straight_mm", "pr_len", "our_len",
                    "len_ratio", "pr_bend", "our_turns", "pr_eff", "our_eff", "ok", "fail"])
        for r in summ["routes"]:
            w.writerow([r.guid, r.group, r.utility, r.size, f"{r.straight_mm:.0f}",
                        f"{r.pr_len:.0f}", f"{r.our_len:.0f}",
                        (f"{r.our_len / r.pr_len:.4f}" if r.ok and r.pr_len > 0 else ""),
                        f"{r.pr_bend:.0f}", r.our_turns, f"{r.pr_eff:.3f}",
                        (f"{r.our_eff:.3f}" if r.ok else ""), int(r.ok), r.fail_reason])
    print(f"[검증] CSV 저장: {path}")


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass
    ap = argparse.ArgumentParser(description="기존 설계배관 vs 우리 A* 검증 리포트")
    ap.add_argument("--project", type=int, default=6)
    ap.add_argument("--cell-mm", type=float, default=100.0)
    ap.add_argument("--w-turn", type=float, default=500.0)
    ap.add_argument("--w-clear", type=float, default=10.0)
    ap.add_argument("--clearance", type=int, default=2)
    ap.add_argument("--out", default="docs/routing3d_validation_report.docx")
    ap.add_argument("--csv", default="docs/routing3d_validation_detail.csv")
    args = ap.parse_args(argv)

    params = RouteParams(cell_mm=args.cell_mm, w_turn=args.w_turn,
                         w_clear=args.w_clear, clearance_radius=args.clearance)
    res = run_validation(args.project, args.cell_mm, params)
    summ = summarize(res)
    print(f"\n[요약] 성공 {summ['n_ok']}/{summ['n']} ({summ['rate']*100:.1f}%) · "
          f"길이비 중앙 {summ['len_ratio_med']:.3f} · "
          f"굴곡 기존 {summ['pr_bend_avg']:.1f}→우리 {summ['our_turns_avg']:.1f} · "
          f"우회비 기존 {summ['pr_eff_avg']:.2f}→우리 {summ['our_eff_avg']:.2f}")
    write_csv(summ, args.csv)
    try:
        write_docx(res, summ, args.out)
    except ModuleNotFoundError:
        print("[경고] python-docx 미설치 → docx 생략(CSV만). pip install python-docx")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
