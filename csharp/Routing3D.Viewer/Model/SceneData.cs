// 씬 데이터 모델 — scene.txt 의 격자/장애물/작업(렌더 입력)
// =============================================================================
//   경로(path)는 C++ 엔진(routing3d_capi)으로부터 받으므로 여기엔 두지 않는다.
//   장애물/격자/작업만 보관해 박스·셀→월드 변환·유틸리티 색에 쓴다.
// =============================================================================
using System.Collections.Generic;

namespace Routing3D.Viewer.Model
{
    /// <summary>격자 메타(셀 크기/원점/셀 개수). 단위 mm.</summary>
    public sealed class GridMeta
    {
        public double CellMm { get; set; } = 50.0;
        public double Ox { get; set; }
        public double Oy { get; set; }
        public double Oz { get; set; }
        public int Nx { get; set; } = 1;
        public int Ny { get; set; } = 1;
        public int Nz { get; set; } = 1;
    }

    /// <summary>장애물 AABB(mm).</summary>
    public sealed class ObstacleBox
    {
        public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    }

    /// <summary>라우팅 작업(start→end, 유틸리티 메타).</summary>
    public sealed class TaskInfo
    {
        public double Sx, Sy, Sz, Gx, Gy, Gz;
        public string? Utility { get; set; }
        public string? Group { get; set; }

        /// <summary>유틸리티 라벨 "[그룹] 유틸"(None/빈 → ?). Python utility_label 과 동일.</summary>
        public string UtilityLabel =>
            $"[{(string.IsNullOrEmpty(Group) ? "?" : Group)}] {(string.IsNullOrEmpty(Utility) ? "?" : Utility)}";
    }

    /// <summary>scene.txt 한 개의 렌더 입력(격자/장애물/작업 + 원문).</summary>
    public sealed class SceneData
    {
        public GridMeta Grid { get; set; } = new();
        public List<ObstacleBox> Obstacles { get; } = new();
        public List<TaskInfo> Tasks { get; } = new();
        public string RawText { get; set; } = string.Empty;
    }
}
