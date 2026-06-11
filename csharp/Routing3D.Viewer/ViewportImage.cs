// 3D 뷰포트(또는 임의 WPF 요소)를 PNG/JPG 이미지 파일로 저장하는 공용 헬퍼.
// 메인 뷰(MainWindow.View)와 진행 다이얼로그 미니 뷰(RoutingProgressWindow.MiniView)가 공유한다.
//
// 원리: WPF 3D(Viewport3D)는 리테인드 모드라 RenderTargetBitmap 이 화면과 동일하게 오프스크린 렌더한다.
//       표시 크기(DIP)는 그대로 두고 DPI 를 96×scale 로 높여 같은 레이아웃을 scale 배 해상도로 슈퍼샘플.
// =============================================================================
using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace Routing3D.Viewer
{
    public static class ViewportImage
    {
        /// <summary>요소를 PNG/JPG 로 저장. 저장 대화상자로 경로·형식을 사용자가 고른다.
        /// scale=2 → 표시 크기의 2배 해상도(선명). 요소가 아직 안 그려졌으면(크기 0) 안내 후 무시.</summary>
        public static void Save(FrameworkElement element, string baseName, double scale = 2.0)
        {
            if (element == null) return;
            int w = (int)Math.Ceiling(element.ActualWidth);
            int h = (int)Math.Ceiling(element.ActualHeight);
            if (w <= 0 || h <= 0)
            {
                MessageBox.Show("뷰가 아직 표시되지 않아 저장할 수 없습니다.", "이미지 저장",
                                MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            var dlg = new Microsoft.Win32.SaveFileDialog
            {
                Title = "3D 뷰 이미지 저장",
                Filter = "PNG 이미지 (*.png)|*.png|JPEG 이미지 (*.jpg)|*.jpg",
                FileName = $"{baseName}_{DateTime.Now:yyyyMMdd_HHmmss}.png",
                DefaultExt = ".png",
            };
            if (dlg.ShowDialog() != true) return;

            try
            {
                // 표시 크기는 그대로(DIP), DPI 를 96×scale 로 높여 동일 화면을 고해상도로 렌더.
                var rtb = new RenderTargetBitmap(
                    (int)(w * scale), (int)(h * scale), 96 * scale, 96 * scale, PixelFormats.Pbgra32);
                rtb.Render(element);

                bool jpg = dlg.FileName.EndsWith(".jpg", StringComparison.OrdinalIgnoreCase)
                        || dlg.FileName.EndsWith(".jpeg", StringComparison.OrdinalIgnoreCase);
                BitmapEncoder enc = jpg ? new JpegBitmapEncoder { QualityLevel = 92 } : new PngBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(rtb));
                using var fs = File.Create(dlg.FileName);
                enc.Save(fs);
            }
            catch (Exception ex)
            {
                MessageBox.Show("이미지 저장 실패: " + ex.Message, "이미지 저장",
                                MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        /// <summary>요소를 캡처해 '클립보드'에 이미지로 복사한다(채팅/문서에 바로 붙여넣기용). 불투명 배경으로
        /// 합성(투명→검정 방지) + 표준 비트맵·PNG 둘 다 담아 브라우저/앱 호환을 넓힌다. scale=2 → 2배 해상도.
        /// 성공/실패를 반환(호출자가 상태바에 안내). 요소가 아직 안 그려졌으면(크기 0) false.</summary>
        public static bool CopyToClipboard(FrameworkElement element, double scale = 2.0)
        {
            if (element == null) return false;
            int w = (int)Math.Ceiling(element.ActualWidth);
            int h = (int)Math.Ceiling(element.ActualHeight);
            if (w <= 0 || h <= 0)
            {
                MessageBox.Show("뷰가 아직 표시되지 않아 복사할 수 없습니다.", "클립보드 복사",
                                MessageBoxButton.OK, MessageBoxImage.Information);
                return false;
            }
            try
            {
                // 1) 화면과 동일 레이아웃을 scale 배 해상도로 렌더(DPI 96×scale → DIP 크기는 w×h 유지).
                var rtb = new RenderTargetBitmap(
                    (int)(w * scale), (int)(h * scale), 96 * scale, 96 * scale, PixelFormats.Pbgra32);
                rtb.Render(element);

                // 2) 불투명 다크 배경(#1e2230) 위에 합성 — 3D 빈 영역의 투명이 붙여넣기 시 검정으로 깨지지 않게.
                var bg = new SolidColorBrush(Color.FromRgb(0x1e, 0x22, 0x30));
                var dv = new DrawingVisual();
                using (var dc = dv.RenderOpen())
                {
                    dc.DrawRectangle(bg, null, new Rect(0, 0, w, h));
                    dc.DrawImage(rtb, new Rect(0, 0, w, h));   // rtb 의 DIP 크기 = w×h.
                }
                var flat = new RenderTargetBitmap(
                    (int)(w * scale), (int)(h * scale), 96 * scale, 96 * scale, PixelFormats.Pbgra32);
                flat.Render(dv);
                flat.Freeze();

                // 3) 표준 비트맵 + PNG 스트림 둘 다 담는다(브라우저/채팅은 PNG 를, 일반 앱은 비트맵을 집어간다).
                var data = new DataObject();
                data.SetImage(flat);
                var png = new PngBitmapEncoder();
                png.Frames.Add(BitmapFrame.Create(flat));
                var ms = new MemoryStream();
                png.Save(ms);
                ms.Position = 0;
                data.SetData("PNG", ms);

                // 4) 클립보드는 다른 프로세스가 잠시 잠글 수 있어 몇 번 재시도(WPF 권장 패턴).
                for (int i = 0; i < 6; i++)
                {
                    try { Clipboard.SetDataObject(data, true); return true; }
                    catch { System.Threading.Thread.Sleep(60); }
                }
                return false;
            }
            catch (Exception ex)
            {
                MessageBox.Show("클립보드 복사 실패: " + ex.Message, "클립보드 복사",
                                MessageBoxButton.OK, MessageBoxImage.Error);
                return false;
            }
        }
    }
}
