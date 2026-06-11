// 기존설계 배관의 출발/종단 스텁(수직배관 + 첫 엘보) 추출 — Python pattern_learn._walk_stub 미러
// =============================================================================
// [이 파일이 하는 일]
//   기존 설계배관(TB_ROUTE_PATH) 폴리라인의 양 끝 '스텁'(PoC 에서 나오는 수직배관 + 수직→수평
//   전환 엘보)을 잘라낸다. 학습 파이프라인(routing3d_py.pattern_learn)과 1:1 동일 로직이라,
//   뷰어가 표시하는 스텁이 학습 데이터의 스텁과 정확히 일치한다.
//
// [알고리즘]  (단위 mm)
//   ① 방향 런 압축(_dirRuns)  : 세그먼트를 6직교 축으로 스냅해 연속 동일 방향을 병합.
//   ② 지터 흡수(_mergeShort)  : MinDirRun(250mm) 미만 런을 인접 런에 흡수(설계 지터 제거).
//   ③ 엘보 포함(WalkStub)     : 첫 런 축=수직축, 축이 다른 첫 런=엘보 → 엘보 + 짧은 리드인
//                                (LeadIn=800mm)까지를 스텁으로. 엘보 없으면 Max/MaxBends 한도.
// =============================================================================
using System;
using System.Collections.Generic;

namespace Routing3D.Viewer.Model
{
    public static class StubExtractor
    {
        private const double MaxMm = 4000.0;       // 스텁 최대 누적 길이.
        private const int MaxBends = 3;            // 엘보 없을 때 최대 방향 수 한도(꺾임 3).
        private const double MinDirRunMm = 250.0;  // 이보다 짧은 방향 런 = 설계 지터(흡수).
        private const double LeadInMm = 800.0;     // 엘보 이후 수평 리드인.

        // 3-벡터를 가장 큰 절대성분 축의 부호로 6직교 축(0..5 = +x,-x,+y,-y,+z,-z)에 스냅.
        //   엔진 NEIGHBORS_6 인덱스와 동일 규약 → r3d_set_task_goal_dir(목표 진입축 제약) 에 그대로 쓴다.
        public static int AxisSnap(double dx, double dy, double dz)
        {
            double ax = Math.Abs(dx), ay = Math.Abs(dy), az = Math.Abs(dz);
            int axis; double val;
            if (ax >= ay && ax >= az) { axis = 0; val = dx; }
            else if (ay >= az) { axis = 1; val = dy; }
            else { axis = 2; val = dz; }
            return axis * 2 + (val >= 0 ? 0 : 1);
        }

        private static double Dist(Pt3 a, Pt3 b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private static Pt3 Lerp(Pt3 a, Pt3 b, double t)
            => new Pt3(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);

        private static List<(int d, double len)> DirRuns(IReadOnlyList<Pt3> seg)
        {
            var runs = new List<(int d, double len)>();
            for (int i = 1; i < seg.Count; i++)
            {
                double L = Dist(seg[i - 1], seg[i]);
                if (L < 1e-6) continue;
                int d = AxisSnap(seg[i].X - seg[i - 1].X, seg[i].Y - seg[i - 1].Y, seg[i].Z - seg[i - 1].Z);
                if (runs.Count > 0 && runs[^1].d == d) runs[^1] = (d, runs[^1].len + L);
                else runs.Add((d, L));
            }
            return runs;
        }

        private static List<(int d, double len)> MergeShort(List<(int d, double len)> runs)
        {
            runs = new List<(int, double)>(runs);
            while (runs.Count > 1)
            {
                int idx = 0;
                for (int i = 1; i < runs.Count; i++) if (runs[i].len < runs[idx].len) idx = i;
                if (runs[idx].len >= MinDirRunMm) break;
                if (idx == 0)
                {
                    runs[1] = (runs[1].d, runs[1].len + runs[0].len);
                    runs.RemoveAt(0);
                }
                else if (idx == runs.Count - 1)
                {
                    runs[^2] = (runs[^2].d, runs[^2].len + runs[^1].len);
                    runs.RemoveAt(runs.Count - 1);
                }
                else if (runs[idx - 1].d == runs[idx + 1].d)
                {
                    runs[idx - 1] = (runs[idx - 1].d, runs[idx - 1].len + runs[idx].len + runs[idx + 1].len);
                    runs.RemoveRange(idx, 2);
                }
                else if (runs[idx - 1].len >= runs[idx + 1].len)
                {
                    runs[idx - 1] = (runs[idx - 1].d, runs[idx - 1].len + runs[idx].len);
                    runs.RemoveAt(idx);
                }
                else
                {
                    runs[idx + 1] = (runs[idx + 1].d, runs[idx + 1].len + runs[idx].len);
                    runs.RemoveAt(idx);
                }
            }
            return runs;
        }

        private static List<Pt3> PointsUntil(IReadOnlyList<Pt3> seg, double length)
        {
            var outp = new List<Pt3> { seg[0] };
            double total = 0;
            for (int i = 1; i < seg.Count; i++)
            {
                double L = Dist(seg[i - 1], seg[i]);
                if (L < 1e-6) continue;
                if (total + L >= length)
                {
                    double t = L > 0 ? (length - total) / L : 1.0;
                    outp.Add(Lerp(seg[i - 1], seg[i], Math.Max(0.0, Math.Min(1.0, t))));
                    break;
                }
                outp.Add(seg[i]);
                total += L;
            }
            return outp;
        }

        /// <summary>PoC 가 front 인 점열에서 스텁(수직배관 + 첫 엘보 + 짧은 리드인) 점열을 잘라 반환.</summary>
        public static List<Pt3> WalkStub(IReadOnlyList<Pt3> seg)
        {
            if (seg.Count < 2) return new List<Pt3>(seg);
            var runs = MergeShort(DirRuns(seg));
            if (runs.Count == 0) return new List<Pt3> { seg[0] };
            int vertAxis = runs[0].d / 2;
            int elbow = -1;
            for (int i = 1; i < runs.Count; i++)
                if (runs[i].d / 2 != vertAxis) { elbow = i; break; }
            double length;
            if (elbow < 0)
            {
                int n = Math.Min(runs.Count, MaxBends + 1);
                double s = 0; for (int i = 0; i < n; i++) s += runs[i].len;
                length = Math.Min(MaxMm, s);
            }
            else
            {
                double pre = 0; for (int i = 0; i < elbow; i++) pre += runs[i].len;
                length = Math.Min(MaxMm, pre + Math.Min(runs[elbow].len, LeadInMm));
            }
            return PointsUntil(seg, length);
        }

        /// <summary>기존배관의 출발/종단 스텁 점열을 반환. SourcePos 가 있으면 그쪽을 front 로 정렬한다.</summary>
        public static (List<Pt3> start, List<Pt3> end) ForPipe(ExistingPipe pipe)
        {
            var pts = pipe.Points;
            if (pts.Count < 2) return (new List<Pt3>(), new List<Pt3>());
            // SourcePos 에 가까운 끝이 front 가 되도록 정렬(없으면 원본 순서).
            var ordered = new List<Pt3>(pts);
            if (pipe.SourcePos is Pt3 sp &&
                Dist(pts[0], sp) > Dist(pts[^1], sp))
                ordered.Reverse();
            var rev = new List<Pt3>(ordered);
            rev.Reverse();
            return (WalkStub(ordered), WalkStub(rev));
        }
    }
}
