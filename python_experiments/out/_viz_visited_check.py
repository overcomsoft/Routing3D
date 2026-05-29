import sys
sys.stdout.reconfigure(encoding="utf-8")
from routing3d_py import DenseOccupancyMap, AABB, astar
from routing3d_py.viz import render_occupancy

occ = DenseOccupancyMap((40, 40, 5), origin=(0, 0, 0), cell_mm=50)
occ.add_box(AABB((500.0, 500.0, 0.0), (1500.0, 1500.0, 250.0)))  # 가운데 벽
res = astar(occ, occ.to_cell((100, 100, 125)), occ.to_cell((1900, 1900, 125)),
            collect_visited=True)
print(res.summary(), "| visited cells =", len(res.visited))
render_occupancy({"obstacles": occ}, opacity=0.3, path=res.path, visited=res.visited,
                 screenshot="out/route_visited.png")
print("OK")
