import cProfile
import io
import pstats
import sys
import time

sys.stdout.reconfigure(encoding="utf-8")

from routing3d_py import AABB, DenseOccupancyMap, RouteParams, astar, astar_weighted

# 대표 그리드: 120x120x60 = 864K 셀, 가운데 장애물 벽들로 우회 유도.
occ = DenseOccupancyMap((120, 120, 60), cell_mm=50.0)
for cx in range(20, 110, 25):
    occ.add_box(AABB((cx * 50.0, 1000.0, 0.0), (cx * 50.0 + 200.0, 5000.0, 2500.0)))
start = occ.to_cell((50.0, 50.0, 50.0))
goal = occ.to_cell((5950.0, 5950.0, 2950.0))
print(f"grid {occ.shape}  blocked {occ.count_blocked():,}")


def bench(fn, n=5):
    best = 1e9
    for _ in range(n):
        t = time.perf_counter()
        r = fn()
        best = min(best, (time.perf_counter() - t) * 1000)
    return best, r


t_a, r_a = bench(lambda: astar(occ, start, goal))
print(f"[astar]          {r_a.summary()}")
print(f"  best wall = {t_a:.1f} ms, 확장 {r_a.expanded_nodes:,}")

params = RouteParams(cell_mm=50.0, w_turn=500.0, w_clear=10.0, clearance_radius=2)
t_w, r_w = bench(lambda: astar_weighted(occ, start, goal, params))
print(f"[astar_weighted] {r_w.summary()}")
print(f"  best wall = {t_w:.1f} ms, 확장 {r_w.expanded_nodes:,}")

print("\n===== cProfile: astar_weighted (tottime 상위) =====")
pr = cProfile.Profile()
pr.enable()
astar_weighted(occ, start, goal, params)
pr.disable()
s = io.StringIO()
pstats.Stats(pr, stream=s).sort_stats("tottime").print_stats(14)
print(s.getvalue())
