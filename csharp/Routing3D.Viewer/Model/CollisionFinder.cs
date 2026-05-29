// 경로 충돌 탐지 — 여러 배관이 같은 셀을 공유하는지(P3)
// =============================================================================
//   다중 순차 라우팅(route_multi)은 충돌 0 을 보장하지만, 단일 재라우팅(route_task)은
//   다른 배관을 무시하므로 겹칠 수 있다. 렌더된 경로 셀에서 ≥2 배관이 공유하는 셀을 찾는다.
// =============================================================================
using System.Collections.Generic;
using Routing3D.Viewer.Interop;

namespace Routing3D.Viewer.Model
{
    public static class CollisionFinder
    {
        /// <summary>여러 경로(셀 배열)에서 ≥2 경로가 공유하는 셀을 반환한다.</summary>
        public static List<(int I, int J, int K)> Find(IEnumerable<PathCell[]> paths)
        {
            var owner = new Dictionary<(int, int, int), int>();
            var collide = new HashSet<(int, int, int)>();
            int t = 0;
            foreach (var path in paths)
            {
                foreach (var c in path)
                {
                    var key = (c.I, c.J, c.K);
                    if (owner.TryGetValue(key, out int prev))
                    {
                        if (prev != t) collide.Add(key);
                    }
                    else
                    {
                        owner[key] = t;
                    }
                }
                t++;
            }
            return new List<(int, int, int)>(collide);
        }
    }
}
