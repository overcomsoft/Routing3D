// scene.txt(v1) 파서 — 격자/장애물/작업 추출(렌더용)
// =============================================================================
//   규격: docs/spec/scene_format_spec.md. 단순 상태기계로 [grid]/[obstacles]/[tasks]
//   섹션만 읽는다(경로 결과는 엔진에서 받으므로 무시). TAB 단일 구분, \N=null, repr 실수.
// =============================================================================
using System;
using System.Globalization;
using Routing3D.Viewer.Model;

namespace Routing3D.Viewer.Model
{
    public static class SceneTextParser
    {
        private const string Null = "\\N";

        public static SceneData Parse(string text)
        {
            var data = new SceneData { RawText = text };
            string section = string.Empty;

            foreach (var raw in text.Split('\n'))
            {
                var line = raw.TrimEnd('\r');
                if (line.Length == 0 || line[0] == '#' || line[0] == '@')
                    continue;

                if (line[0] == '[')
                {
                    // "[name]\tk=v..." → 섹션 이름만 취한다.
                    var head = line.Split('\t')[0];
                    section = head.Substring(1, head.IndexOf(']') - 1);
                    continue;
                }

                var cols = line.Split('\t');
                switch (section)
                {
                    case "grid":
                        ParseGrid(data.Grid, cols);
                        break;
                    case "obstacles":
                        if (cols.Length >= 6) data.Obstacles.Add(ParseObstacle(cols));
                        break;
                    case "tasks":
                        if (cols.Length >= 6) data.Tasks.Add(ParseTask(cols));
                        break;
                    // params/results/result/path/visited 는 렌더에 불필요 → 무시.
                }
            }
            return data;
        }

        private static void ParseGrid(GridMeta g, string[] c)
        {
            switch (c[0])
            {
                case "cell_mm": g.CellMm = D(c[1]); break;
                case "origin": g.Ox = D(c[1]); g.Oy = D(c[2]); g.Oz = D(c[3]); break;
                case "shape": g.Nx = I(c[1]); g.Ny = I(c[2]); g.Nz = I(c[3]); break;
            }
        }

        private static ObstacleBox ParseObstacle(string[] c) => new()
        {
            MinX = D(c[0]), MinY = D(c[1]), MinZ = D(c[2]),
            MaxX = D(c[3]), MaxY = D(c[4]), MaxZ = D(c[5])
        };

        private static TaskInfo ParseTask(string[] c) => new()
        {
            Sx = D(c[0]), Sy = D(c[1]), Sz = D(c[2]),
            Gx = D(c[3]), Gy = D(c[4]), Gz = D(c[5]),
            Utility = c.Length > 6 ? Opt(c[6]) : null,
            Group = c.Length > 7 ? Opt(c[7]) : null
        };

        private static string? Opt(string token) => token == Null ? null : token;
        private static double D(string s) => double.Parse(s, CultureInfo.InvariantCulture);
        private static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);
    }
}
