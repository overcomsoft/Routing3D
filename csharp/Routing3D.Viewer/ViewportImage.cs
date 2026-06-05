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
    }
}
