// 유틸리티 라벨 → 색 (결정적 배정) — Python routing3d_py.scene.utility_colors 와 동일 규약
// =============================================================================
//   라벨 목록을 정렬(Ordinal) 후 팔레트를 순환 배정한다. 같은 입력 → 같은 색(결정적).
// =============================================================================
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;

namespace Routing3D.Viewer.Model
{
    public static class UtilityColors
    {
        // Python _PALETTE 와 동일 순서/색.
        private static readonly Color[] Palette =
        {
            Colors.Red, Colors.Blue, Colors.Green, Colors.Orange, Colors.Purple,
            Colors.DeepPink, Colors.Teal, Colors.Gold, Colors.SaddleBrown, Colors.Cyan,
            Colors.Magenta, Colors.LimeGreen, Colors.Navy, Colors.Crimson, Colors.DarkOrange,
            Colors.MediumSpringGreen, Colors.SlateBlue, Colors.Tomato, Colors.SeaGreen, Colors.RoyalBlue,
            Colors.Violet, Colors.Olive, Colors.IndianRed, Colors.Turquoise
        };

        public static Dictionary<string, Color> Assign(IEnumerable<string> labels)
        {
            var sorted = labels.Distinct().OrderBy(x => x, StringComparer.Ordinal).ToList();
            var map = new Dictionary<string, Color>();
            for (int i = 0; i < sorted.Count; i++)
                map[sorted[i]] = Palette[i % Palette.Length];
            return map;
        }
    }
}
