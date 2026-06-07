// 오프스크린 3D 스냅샷 렌더러 — 창 없이 Viewport3D 를 PNG 로 굽는다(P4).
// =============================================================================
// [무엇]
//   AI 자동설계 비교 리포트(AutoDesignReport)가 케이스별 3전략(기존/최단/Stub+그룹)을
//   '같은 카메라'로 3장 렌더해 시각 비교를 만들 때 쓰는 공용 헬퍼.
//
// [원리]
//   WPF 3D(Viewport3D)는 리테인드 모드라 화면에 붙이지 않아도 RenderTargetBitmap 이
//   오프스크린(소프트웨어 폴백)으로 동일하게 렌더한다. 새로 만든 비주얼은 Measure→Arrange→
//   UpdateLayout 으로 수동 레이아웃한 뒤 Render 하면 된다. 반드시 STA 스레드에서 호출
//   (App.OnStartup = STA 이므로 --autodesign-report 경로에서 그대로 호출 가능).
//
// [좌표계]
//   데이터는 Z-업(Z=높이) mm. PerspectiveCamera UpDirection=(0,0,1), 카메라는 +X+Y+Z
//   방향(아이소메트릭)에서 중심을 바라본다. 한 케이스의 3장은 동일 bounds 를 받아 카메라가
//   같으므로 길이·다발화 차이를 그대로 눈으로 비교할 수 있다.
//
// [재사용]  HelixToolkit.Wpf MeshBuilder(박스=AddBox, 배관=AddTube+AddSphere). 색/머티리얼은
//   SceneViewModel 의 Geometry/MaterialFor 와 동일 규약(DiffuseMaterial, 앞뒷면 동일).
// =============================================================================
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace Routing3D.Viewer.Diagnostics
{
    public static class OffscreenRenderer
    {
        /// <summary>한 배관(폴리라인) — 월드 mm 점열 + 색.</summary>
        public sealed class Poly
        {
            public List<Point3D> Pts = new();
            public Color Color = Colors.Gray;
        }

        /// <summary>맥락 박스(장비/덕트 등) — 중심·치수(mm) + 색/알파.</summary>
        public sealed class Box
        {
            public Point3D Center;
            public double Dx, Dy, Dz;
            public Color Color = Colors.Gray;
            public byte Alpha = 60;
        }

        /// <summary>박스+배관을 PNG 로 렌더. bounds=카메라 프레이밍 기준(케이스 3장 공유 → 동일 시점).
        /// title/subtitle 은 좌상단에 구워 단독 이미지로도 식별되게 한다.</summary>
        public static void RenderToPng(string path, int w, int h,
                                       IEnumerable<Box> boxes, IEnumerable<Poly> polys,
                                       Rect3D bounds, string? title, string? subtitle)
        {
            double sx = Math.Max(bounds.SizeX, 1), sy = Math.Max(bounds.SizeY, 1), sz = Math.Max(bounds.SizeZ, 1);
            double diag = Math.Sqrt(sx * sx + sy * sy + sz * sz);
            double tubeDia = Math.Max(40.0, diag * 0.006);   // 화면에서 보이는 배관 굵기(데이터 굵기 무관, 시각용).
            double markerR = tubeDia * 1.6;

            // ---- 지오메트리 그룹(색별 머지 메시) ----
            var model = new Model3DGroup();

            var boxByKey = new Dictionary<(uint, byte), MeshBuilder>();
            foreach (var b in boxes)
            {
                var key = (ColorKey(b.Color), b.Alpha);
                if (!boxByKey.TryGetValue(key, out var mb)) { mb = new MeshBuilder(false, false); boxByKey[key] = mb; }
                mb.AddBox(b.Center, b.Dx, b.Dy, b.Dz);
            }
            foreach (var kv in boxByKey)
            {
                var (ck, alpha) = kv.Key;
                model.Children.Add(Geom(kv.Value, FromKey(ck), alpha));
            }

            var tubeByColor = new Dictionary<uint, MeshBuilder>();
            foreach (var p in polys)
            {
                if (p.Pts.Count < 1) continue;
                var ck = ColorKey(p.Color);
                if (!tubeByColor.TryGetValue(ck, out var mb)) { mb = new MeshBuilder(false, false); tubeByColor[ck] = mb; }
                if (p.Pts.Count >= 2) mb.AddTube(p.Pts, tubeDia, 8, false);
                mb.AddSphere(p.Pts[0], markerR);
                mb.AddSphere(p.Pts[p.Pts.Count - 1], markerR);
            }
            foreach (var kv in tubeByColor)
                model.Children.Add(Geom(kv.Value, FromKey(kv.Key), 255));

            // ---- 라이트 ----
            model.Children.Add(new AmbientLight(Color.FromRgb(110, 110, 110)));
            model.Children.Add(new DirectionalLight(Color.FromRgb(210, 210, 210), new Vector3D(-1, -1, -2)));
            model.Children.Add(new DirectionalLight(Color.FromRgb(120, 120, 120), new Vector3D(1, 1, -0.4)));

            // ---- 카메라(아이소메트릭, Z-업) ----
            var center = new Point3D(bounds.X + sx / 2, bounds.Y + sy / 2, bounds.Z + sz / 2);
            double radius = diag / 2;
            double fov = 45.0;
            double dist = radius / Math.Tan(fov * Math.PI / 360.0) * 1.35;
            var offset = new Vector3D(1, 1, 0.85); offset.Normalize();
            var cam = new PerspectiveCamera
            {
                Position = center + offset * dist,
                LookDirection = -offset * dist,
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = fov,
                NearPlaneDistance = Math.Max(1.0, diag * 0.005),
                FarPlaneDistance = diag * 12,
            };

            var vp = new Viewport3D { Width = w, Height = h, Camera = cam, ClipToBounds = true };
            vp.Children.Add(new ModelVisual3D { Content = model });

            // ---- 배경 + 타이틀 오버레이 ----
            var root = new Grid { Width = w, Height = h, Background = new SolidColorBrush(Color.FromRgb(248, 249, 251)) };
            root.Children.Add(vp);
            if (!string.IsNullOrEmpty(title))
            {
                var sp = new StackPanel { Margin = new Thickness(10, 8, 0, 0),
                                          HorizontalAlignment = HorizontalAlignment.Left,
                                          VerticalAlignment = VerticalAlignment.Top };
                sp.Children.Add(new TextBlock { Text = title, FontSize = 16, FontWeight = FontWeights.Bold,
                                                Foreground = new SolidColorBrush(Color.FromRgb(40, 50, 70)) });
                if (!string.IsNullOrEmpty(subtitle))
                    sp.Children.Add(new TextBlock { Text = subtitle, FontSize = 12,
                                                    Foreground = new SolidColorBrush(Color.FromRgb(90, 100, 120)) });
                root.Children.Add(sp);
            }

            var size = new Size(w, h);
            root.Measure(size);
            root.Arrange(new Rect(size));
            root.UpdateLayout();

            var rtb = new RenderTargetBitmap(w, h, 96, 96, PixelFormats.Pbgra32);
            rtb.Render(root);

            var enc = new PngBitmapEncoder();
            enc.Frames.Add(BitmapFrame.Create(rtb));
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            using var fs = File.Create(path);
            enc.Save(fs);
        }

        /// <summary>여러 폴리라인의 점 + 추가 점들을 모두 감싸는 AABB(여백 포함). 빈 입력이면 null.</summary>
        public static Rect3D? ComputeBounds(IEnumerable<Poly> polys, double marginMm)
        {
            double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue;
            double mxx = double.MinValue, mxy = double.MinValue, mxz = double.MinValue;
            bool any = false;
            foreach (var p in polys)
                foreach (var pt in p.Pts)
                {
                    any = true;
                    if (pt.X < mnx) mnx = pt.X; if (pt.Y < mny) mny = pt.Y; if (pt.Z < mnz) mnz = pt.Z;
                    if (pt.X > mxx) mxx = pt.X; if (pt.Y > mxy) mxy = pt.Y; if (pt.Z > mxz) mxz = pt.Z;
                }
            if (!any) return null;
            mnx -= marginMm; mny -= marginMm; mnz -= marginMm;
            mxx += marginMm; mxy += marginMm; mxz += marginMm;
            return new Rect3D(mnx, mny, mnz, mxx - mnx, mxy - mny, mxz - mnz);
        }

        private static GeometryModel3D Geom(MeshBuilder mb, Color color, byte alpha)
        {
            var c = Color.FromArgb(alpha, color.R, color.G, color.B);
            var mat = new DiffuseMaterial(new SolidColorBrush(c));
            return new GeometryModel3D { Geometry = mb.ToMesh(), Material = mat, BackMaterial = mat };
        }

        private static uint ColorKey(Color c) => ((uint)c.R << 16) | ((uint)c.G << 8) | c.B;
        private static Color FromKey(uint k) => Color.FromRgb((byte)(k >> 16), (byte)(k >> 8), (byte)k);
    }
}
