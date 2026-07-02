using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HelixToolkit.Wpf;
using Microsoft.Win32;

namespace Routing3D.Viewer.Views
{
    public partial class TraceReplayWindow : Window
    {
        public ObservableCollection<TraceEventRow> Events { get; } = new();

        public TraceEventRow? SelectedEvent { get; set; }

        private readonly ICollectionView _eventsView;
        private readonly DispatcherTimer _playTimer;
        private double _cellMm = 25.0;
        private int _nx, _ny, _nz;
        private double _ox, _oy, _oz;
        private string _path = string.Empty;
        private bool _isPlaying;
        private Rect3D? _lastFocusBounds;
        private readonly List<TraceCell> _occupancyCells = new();
        private readonly List<TraceCell> _passthroughCells = new();
        private bool _savingVideo;
        private bool _uiReady;
        private int _pathPlaybackCells;

        public TraceReplayWindow(string? path = null)
        {
            InitializeComponent();
            _uiReady = true;
            DataContext = this;

            _eventsView = CollectionViewSource.GetDefaultView(Events);
            _eventsView.Filter = FilterEvent;

            _playTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(350) };
            _playTimer.Tick += OnPlaybackTick;

            TypeFilterCombo.ItemsSource = new[] { "All" };
            TypeFilterCombo.SelectedIndex = 0;
            SpeedText.Text = ((int)PlaybackSpeedSlider.Value).ToString();

            if (!string.IsNullOrWhiteSpace(path) && File.Exists(path))
                LoadTrace(path);
            else
                UpdateStatus();
        }

        private void OnOpen(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title = "Open Routing3D trace log",
                Filter = "Routing3D trace (*.r3dtrace.jsonl)|*.r3dtrace.jsonl|JSONL (*.jsonl)|*.jsonl|All files (*.*)|*.*",
                InitialDirectory = Directory.Exists(Path.Combine(AppContext.BaseDirectory, "logs"))
                    ? Path.Combine(AppContext.BaseDirectory, "logs")
                    : AppContext.BaseDirectory,
            };
            if (dlg.ShowDialog(this) == true)
                LoadTrace(dlg.FileName);
        }

        private void LoadTrace(string path)
        {
            StopPlayback();
            Events.Clear();
            _occupancyCells.Clear();
            _passthroughCells.Clear();
            _path = path;
            int seq = 0;
            foreach (var line in ReadTraceLines(path))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                try
                {
                    using var doc = JsonDocument.Parse(line);
                    var root = doc.RootElement;
                    var type = GetString(root, "type") ?? "?";
                    if (type == "trace_header") ReadHeader(root);
                    else if (type == "occupancy_sample") ReadCellSample(root, _occupancyCells);
                    else if (type == "passthrough_sample") ReadCellSample(root, _passthroughCells);
                    Events.Add(new TraceEventRow(seq++, type, GetInt(root, "task"), Summarize(root), line));
                }
                catch (Exception ex)
                {
                    Events.Add(new TraceEventRow(seq++, "parse_error", null, ex.Message, line));
                }
            }

            UpdateTypeFilterOptions();
            _eventsView.Refresh();
            UpdateStatus();
            SelectFirstVisible();
        }

        private static void ReadCellSample(JsonElement root, List<TraceCell> target)
        {
            target.Clear();
            if (!TryGetArray(root, "cells", out var cells)) return;
            foreach (var item in cells.EnumerateArray())
            {
                if (item.ValueKind != JsonValueKind.Array || item.GetArrayLength() < 3) continue;
                target.Add(new TraceCell(item[0].GetInt32(), item[1].GetInt32(), item[2].GetInt32()));
            }
        }

        private static IEnumerable<string> ReadTraceLines(string path)
        {
            using var fs = new FileStream(path, FileMode.Open, FileAccess.Read,
                                          FileShare.ReadWrite | FileShare.Delete);
            using var reader = new StreamReader(fs);
            while (reader.ReadLine() is { } line)
                yield return line;
        }

        private bool FilterEvent(object item)
        {
            if (item is not TraceEventRow row) return false;

            var taskText = TaskFilterBox?.Text?.Trim();
            if (!string.IsNullOrEmpty(taskText) &&
                (!row.Task.HasValue || !row.Task.Value.ToString().Contains(taskText, StringComparison.OrdinalIgnoreCase)))
                return false;

            var typeText = TypeFilterCombo?.SelectedItem as string;
            if (!string.IsNullOrWhiteSpace(typeText) && typeText != "All" &&
                !string.Equals(row.Type, typeText, StringComparison.OrdinalIgnoreCase))
                return false;

            var search = SearchBox?.Text?.Trim();
            if (!string.IsNullOrEmpty(search) &&
                !row.Type.Contains(search, StringComparison.OrdinalIgnoreCase) &&
                !row.Summary.Contains(search, StringComparison.OrdinalIgnoreCase) &&
                !row.RawJson.Contains(search, StringComparison.OrdinalIgnoreCase))
                return false;

            return true;
        }

        private void UpdateTypeFilterOptions()
        {
            var selected = TypeFilterCombo.SelectedItem as string ?? "All";
            var items = new List<string> { "All" };
            items.AddRange(Events.Select(e => e.Type).Distinct().OrderBy(x => x, StringComparer.OrdinalIgnoreCase));
            TypeFilterCombo.ItemsSource = items;
            TypeFilterCombo.SelectedItem = items.Contains(selected) ? selected : "All";
        }

        private void UpdateStatus()
        {
            int visible = _eventsView.Cast<TraceEventRow>().Count();
            string name = string.IsNullOrWhiteSpace(_path) ? "No trace" : Path.GetFileName(_path);
            StatusText.Text = $"{name} | visible {visible:N0}/{Events.Count:N0} | occ={_occupancyCells.Count:N0} pass={_passthroughCells.Count:N0} | cell={_cellMm:0.###}mm | grid={_nx}x{_ny}x{_nz}";
        }

        private void ReadHeader(JsonElement root)
        {
            if (TryGetDouble(root, "cell_mm", out var c)) _cellMm = c;
            if (TryGetArray(root, "origin", out var origin) && origin.GetArrayLength() >= 3)
            {
                _ox = origin[0].GetDouble();
                _oy = origin[1].GetDouble();
                _oz = origin[2].GetDouble();
            }
            if (TryGetArray(root, "shape", out var shape) && shape.GetArrayLength() >= 3)
            {
                _nx = shape[0].GetInt32();
                _ny = shape[1].GetInt32();
                _nz = shape[2].GetInt32();
            }
        }

        private void OnEventSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            var row = EventGrid.SelectedItem as TraceEventRow;
            DetailText.Text = row?.RawJson ?? "";
            RebuildModel(row);
        }

        private void OnFilterChanged(object sender, EventArgs e)
        {
            if (_eventsView == null) return;
            _eventsView.Refresh();
            UpdateStatus();
            if (EventGrid.SelectedItem is not TraceEventRow selected || !FilterEvent(selected))
                SelectFirstVisible();
        }

        private void OnClearFilters(object sender, RoutedEventArgs e)
        {
            TaskFilterBox.Text = "";
            SearchBox.Text = "";
            TypeFilterCombo.SelectedItem = "All";
            _eventsView.Refresh();
            UpdateStatus();
            SelectFirstVisible();
        }

        private void RebuildModel(TraceEventRow? row)
        {
            if (!_uiReady || TraceModelVisual == null) return;

            var group = new Model3DGroup();
            var focusPoints = new List<Point3D>();
            if (_nx > 0 && _ny > 0 && _nz > 0)
            {
                var lo = new Point3D(_ox, _oy, _oz);
                var hi = new Point3D(_ox + _nx * _cellMm, _oy + _ny * _cellMm, _oz + _nz * _cellMm);
                AddBoxFrame(group, lo, hi, Color.FromRgb(65, 78, 105), Math.Max(_cellMm * 0.12, 3.0));
                if (ShowVoxelMapBox?.IsChecked == true)
                    AddVoxelMap(group);
            }

            if (ShowOccupancyMapBox?.IsChecked == true)
                AddCellCloud(group, _occupancyCells, Color.FromRgb(105, 112, 128), 0.34, 0.82);
            if (ShowPassthroughMapBox?.IsChecked == true)
                AddCellCloud(group, _passthroughCells, Color.FromRgb(60, 220, 230), 0.42, 0.76);

            if (row != null)
            {
                try
                {
                    using var doc = JsonDocument.Parse(row.RawJson);
                    var eventCells = ExtractEventCells(doc.RootElement);
                    var taskBounds = GetTaskSpace(row);
                    if (taskBounds.HasValue)
                    {
                        AddTaskVoxelSpace(group, taskBounds.Value, focusPoints);
                        if (ShowOccupancyMapBox?.IsChecked == true)
                            AddCellCloud(group, CellsInBounds(_occupancyCells, taskBounds.Value), Color.FromRgb(125, 130, 145), 0.46, 0.58);
                        if (ShowPassthroughMapBox?.IsChecked == true)
                            AddCellCloud(group, CellsInBounds(_passthroughCells, taskBounds.Value), Color.FromRgb(60, 220, 230), 0.52, 0.62);
                    }
                    if (ShowVoxelMapBox?.IsChecked == true)
                        AddLocalVoxelWindow(group, eventCells, focusPoints);
                    AddTraceContext(group, row, taskBounds, focusPoints);
                    AddCellsForEvent(group, doc.RootElement, focusPoints);
                }
                catch
                {
                    // Parse errors are already shown in the detail panel.
                }
            }

            _lastFocusBounds = BuildFocusBounds(focusPoints);
            TraceModelVisual.Content = group;
            FocusCurrentEvent();
        }

        private List<TraceCell> ExtractEventCells(JsonElement root)
        {
            var cells = new List<TraceCell>();
            foreach (string name in new[] { "cell", "from", "to", "source_cell", "target_cell", "snapped_source", "snapped_target" })
                if (TryGetCell(root, name, out var cell)) cells.Add(cell);
            return cells;
        }

        private CellBounds? GetTaskSpace(TraceEventRow selected)
        {
            if (!selected.Task.HasValue) return null;
            var cells = new List<TraceCell>();
            foreach (var row in Events.Where(e => e.Task == selected.Task && e.Seq <= selected.Seq))
            {
                try
                {
                    using var doc = JsonDocument.Parse(row.RawJson);
                    cells.AddRange(ExtractEventCells(doc.RootElement));
                    if (row.Type == "route_path" && TryGetArray(doc.RootElement, "cells", out var pathCells))
                        cells.AddRange(ReadCells(pathCells));
                }
                catch
                {
                    // Ignore malformed rows.
                }
            }

            if (cells.Count == 0) return null;
            int pad = 10;
            return new CellBounds(
                Math.Max(0, cells.Min(c => c.I) - pad), Math.Min(_nx - 1, cells.Max(c => c.I) + pad),
                Math.Max(0, cells.Min(c => c.J) - pad), Math.Min(_ny - 1, cells.Max(c => c.J) + pad),
                Math.Max(0, cells.Min(c => c.K) - pad), Math.Min(_nz - 1, cells.Max(c => c.K) + pad));
        }

        private static IEnumerable<TraceCell> ReadCells(JsonElement cells)
        {
            foreach (var item in cells.EnumerateArray())
                if (item.ValueKind == JsonValueKind.Array && item.GetArrayLength() >= 3)
                    yield return new TraceCell(item[0].GetInt32(), item[1].GetInt32(), item[2].GetInt32());
        }

        private static List<TraceCell> CellsInBounds(IReadOnlyList<TraceCell> cells, CellBounds b)
            => cells.Where(c => c.I >= b.MinI && c.I <= b.MaxI &&
                                c.J >= b.MinJ && c.J <= b.MaxJ &&
                                c.K >= b.MinK && c.K <= b.MaxK).ToList();

        private void AddTaskVoxelSpace(Model3DGroup group, CellBounds b, List<Point3D> focusPoints)
        {
            var lo = new Point3D(_ox + b.MinI * _cellMm, _oy + b.MinJ * _cellMm, _oz + b.MinK * _cellMm);
            var hi = new Point3D(_ox + (b.MaxI + 1) * _cellMm, _oy + (b.MaxJ + 1) * _cellMm, _oz + (b.MaxK + 1) * _cellMm);
            AddBoxFrame(group, lo, hi, Color.FromRgb(120, 150, 210), Math.Max(_cellMm * 0.075, 2.0));

            if (ShowVoxelMapBox?.IsChecked == true)
            {
                var mb = new MeshBuilder(false, false);
                double r = Math.Max(_cellMm * 0.018, 0.8);
                int sx = Math.Max(1, (b.MaxI - b.MinI + 1) / 18);
                int sy = Math.Max(1, (b.MaxJ - b.MinJ + 1) / 18);
                int sz = Math.Max(1, (b.MaxK - b.MinK + 1) / 10);

                for (int i = b.MinI; i <= b.MaxI + 1; i += sx)
                {
                    double x = _ox + i * _cellMm;
                    mb.AddCylinder(new Point3D(x, lo.Y, lo.Z), new Point3D(x, hi.Y, lo.Z), r, 4);
                    mb.AddCylinder(new Point3D(x, lo.Y, hi.Z), new Point3D(x, hi.Y, hi.Z), r, 4);
                }
                for (int j = b.MinJ; j <= b.MaxJ + 1; j += sy)
                {
                    double y = _oy + j * _cellMm;
                    mb.AddCylinder(new Point3D(lo.X, y, lo.Z), new Point3D(hi.X, y, lo.Z), r, 4);
                    mb.AddCylinder(new Point3D(lo.X, y, hi.Z), new Point3D(hi.X, y, hi.Z), r, 4);
                }
                for (int k = b.MinK; k <= b.MaxK + 1; k += sz)
                {
                    double z = _oz + k * _cellMm;
                    mb.AddCylinder(new Point3D(lo.X, lo.Y, z), new Point3D(hi.X, lo.Y, z), r, 4);
                    mb.AddCylinder(new Point3D(lo.X, hi.Y, z), new Point3D(hi.X, hi.Y, z), r, 4);
                    mb.AddCylinder(new Point3D(lo.X, lo.Y, z), new Point3D(lo.X, hi.Y, z), r, 4);
                    mb.AddCylinder(new Point3D(hi.X, lo.Y, z), new Point3D(hi.X, hi.Y, z), r, 4);
                }

                group.Children.Add(new GeometryModel3D
                {
                    Geometry = mb.ToMesh(true),
                    Material = MaterialHelper.CreateMaterial(Color.FromArgb(70, 100, 130, 185)),
                    BackMaterial = MaterialHelper.CreateMaterial(Color.FromArgb(70, 100, 130, 185)),
                });
            }

            focusPoints.Add(lo);
            focusPoints.Add(hi);
        }

        private void AddTraceContext(Model3DGroup group, TraceEventRow selected, CellBounds? taskBounds, List<Point3D> focusPoints)
        {
            if (!selected.Task.HasValue) return;
            var rows = Events
                .Where(e => e.Seq <= selected.Seq && e.Task == selected.Task)
                .Where(e => e.Type is "expand_cell" or "candidate_reject" or "snap" or "task_begin" or "route_path")
                .ToList();
            if (rows.Count == 0) return;

            const int cap = 900;
            if (rows.Count > cap)
            {
                int keepRecent = Math.Min(500, rows.Count);
                int sampleCount = cap - keepRecent;
                var sampled = new List<TraceEventRow>(cap);
                double stride = (double)(rows.Count - keepRecent) / Math.Max(1, sampleCount);
                for (int i = 0; i < sampleCount; i++)
                    sampled.Add(rows[(int)(i * stride)]);
                sampled.AddRange(rows.Skip(rows.Count - keepRecent));
                rows = sampled;
            }

            var expanded = new List<TraceCell>();
            var rejected = new List<TraceCell>();
            var snapped = new List<TraceCell>();
            var finalPath = new List<TraceCell>();

            foreach (var row in rows)
            {
                try
                {
                    using var doc = JsonDocument.Parse(row.RawJson);
                    var root = doc.RootElement;
                    string type = GetString(root, "type") ?? "";
                    if (type == "expand_cell" && TryGetCell(root, "cell", out var ec)) expanded.Add(ec);
                    else if (type == "candidate_reject" && TryGetCell(root, "to", out var rc)) rejected.Add(rc);
                    else if (type == "snap" && TryGetCell(root, "to", out var sc)) snapped.Add(sc);
                    else if (type == "route_path" && TryGetArray(root, "cells", out var pathCells))
                    {
                        var path = ReadCells(pathCells);
                        if (PathPlaybackBox?.IsChecked == true && row.Seq == selected.Seq)
                            path = path.Take(Math.Max(1, _pathPlaybackCells));
                        finalPath.AddRange(path);
                    }
                    else if (type == "task_begin")
                    {
                        if (TryGetCell(root, "snapped_source", out var ss)) snapped.Add(ss);
                        if (TryGetCell(root, "snapped_target", out var st)) snapped.Add(st);
                    }
                }
                catch
                {
                    // Ignore malformed rows; the detail panel still shows the raw parse error.
                }
            }

            AddCellCloud(group, expanded, Color.FromRgb(255, 210, 40), 0.42, 0.55);
            AddCellCloud(group, rejected, Color.FromRgb(255, 80, 40), 0.38, 0.42);
            AddCellCloud(group, snapped, Color.FromRgb(70, 190, 255), 0.58, 0.70);
            AddCellCloud(group, finalPath, Color.FromRgb(40, 255, 120), 0.66, 0.90);
            AddPathTube(group, finalPath, Color.FromRgb(40, 255, 120));

            foreach (var c in expanded.TakeLast(Math.Min(120, expanded.Count)))
                focusPoints.Add(CellCenter(c.I, c.J, c.K));
            foreach (var c in rejected.TakeLast(Math.Min(120, rejected.Count)))
                focusPoints.Add(CellCenter(c.I, c.J, c.K));
            foreach (var c in finalPath)
                focusPoints.Add(CellCenter(c.I, c.J, c.K));
        }

        private void AddPathTube(Model3DGroup group, IReadOnlyList<TraceCell> cells, Color color)
        {
            if (cells.Count < 2) return;
            var pts = cells.Select(c => CellCenter(c.I, c.J, c.K)).ToList();
            var mb = new MeshBuilder(false, false);
            mb.AddTube(pts, Math.Max(_cellMm * 0.38, 10.0), 8, false);
            var mat = MaterialHelper.CreateMaterial(Color.FromArgb(235, color.R, color.G, color.B));
            group.Children.Add(new GeometryModel3D { Geometry = mb.ToMesh(true), Material = mat, BackMaterial = mat });
        }

        private void AddLocalVoxelWindow(Model3DGroup group, IReadOnlyList<TraceCell> eventCells, List<Point3D> focusPoints)
        {
            if (eventCells.Count == 0) return;
            int radius = 7;
            int minI = Math.Max(0, eventCells.Min(c => c.I) - radius);
            int maxI = Math.Min(_nx - 1, eventCells.Max(c => c.I) + radius);
            int minJ = Math.Max(0, eventCells.Min(c => c.J) - radius);
            int maxJ = Math.Min(_ny - 1, eventCells.Max(c => c.J) + radius);
            int minK = Math.Max(0, eventCells.Min(c => c.K) - radius);
            int maxK = Math.Min(_nz - 1, eventCells.Max(c => c.K) + radius);

            var lo = new Point3D(_ox + minI * _cellMm, _oy + minJ * _cellMm, _oz + minK * _cellMm);
            var hi = new Point3D(_ox + (maxI + 1) * _cellMm, _oy + (maxJ + 1) * _cellMm, _oz + (maxK + 1) * _cellMm);
            AddBoxFrame(group, lo, hi, Color.FromRgb(95, 120, 165), Math.Max(_cellMm * 0.05, 1.6));

            var mb = new MeshBuilder(false, false);
            double r = Math.Max(_cellMm * 0.018, 0.8);
            for (int i = minI; i <= maxI + 1; i++)
            {
                double x = _ox + i * _cellMm;
                mb.AddCylinder(new Point3D(x, lo.Y, lo.Z), new Point3D(x, hi.Y, lo.Z), r, 4);
                mb.AddCylinder(new Point3D(x, lo.Y, hi.Z), new Point3D(x, hi.Y, hi.Z), r, 4);
            }
            for (int j = minJ; j <= maxJ + 1; j++)
            {
                double y = _oy + j * _cellMm;
                mb.AddCylinder(new Point3D(lo.X, y, lo.Z), new Point3D(hi.X, y, lo.Z), r, 4);
                mb.AddCylinder(new Point3D(lo.X, y, hi.Z), new Point3D(hi.X, y, hi.Z), r, 4);
            }
            for (int k = minK; k <= maxK + 1; k++)
            {
                double z = _oz + k * _cellMm;
                mb.AddCylinder(new Point3D(lo.X, lo.Y, z), new Point3D(hi.X, lo.Y, z), r, 4);
                mb.AddCylinder(new Point3D(lo.X, hi.Y, z), new Point3D(hi.X, hi.Y, z), r, 4);
                mb.AddCylinder(new Point3D(lo.X, lo.Y, z), new Point3D(lo.X, hi.Y, z), r, 4);
                mb.AddCylinder(new Point3D(hi.X, lo.Y, z), new Point3D(hi.X, hi.Y, z), r, 4);
            }

            group.Children.Add(new GeometryModel3D
            {
                Geometry = mb.ToMesh(true),
                Material = MaterialHelper.CreateMaterial(Color.FromArgb(92, 95, 120, 165)),
                BackMaterial = MaterialHelper.CreateMaterial(Color.FromArgb(92, 95, 120, 165)),
            });

            focusPoints.Add(lo);
            focusPoints.Add(hi);
        }

        private void OnLayerChanged(object sender, RoutedEventArgs e)
        {
            if (!_uiReady) return;
            RebuildModel(EventGrid?.SelectedItem as TraceEventRow);
        }

        private void OnPathPlaybackChanged(object sender, RoutedEventArgs e)
        {
            if (!_uiReady) return;
            StopPlayback();
            _pathPlaybackCells = 0;
            RebuildModel(EventGrid?.SelectedItem as TraceEventRow);
        }

        private void AddVoxelMap(Model3DGroup group)
        {
            if (_nx <= 0 || _ny <= 0 || _nz <= 0) return;
            var mb = new MeshBuilder(false, false);
            int targetLines = 18;
            int sx = Math.Max(1, _nx / targetLines);
            int sy = Math.Max(1, _ny / targetLines);
            int sz = Math.Max(1, _nz / Math.Max(8, targetLines / 2));
            double r = Math.Max(_cellMm * 0.035, 1.2);
            double x0 = _ox, x1 = _ox + _nx * _cellMm;
            double y0 = _oy, y1 = _oy + _ny * _cellMm;
            double z0 = _oz, z1 = _oz + _nz * _cellMm;

            for (int i = 0; i <= _nx; i += sx)
            {
                double x = _ox + i * _cellMm;
                mb.AddCylinder(new Point3D(x, y0, z0), new Point3D(x, y1, z0), r, 4);
            }
            for (int j = 0; j <= _ny; j += sy)
            {
                double y = _oy + j * _cellMm;
                mb.AddCylinder(new Point3D(x0, y, z0), new Point3D(x1, y, z0), r, 4);
            }
            for (int k = sz; k < _nz; k += sz)
            {
                double z = _oz + k * _cellMm;
                mb.AddCylinder(new Point3D(x0, y0, z), new Point3D(x0, y1, z), r, 4);
                mb.AddCylinder(new Point3D(x1, y0, z), new Point3D(x1, y1, z), r, 4);
                mb.AddCylinder(new Point3D(x0, y0, z), new Point3D(x1, y0, z), r, 4);
                mb.AddCylinder(new Point3D(x0, y1, z), new Point3D(x1, y1, z), r, 4);
            }

            group.Children.Add(new GeometryModel3D
            {
                Geometry = mb.ToMesh(true),
                Material = MaterialHelper.CreateMaterial(Color.FromArgb(80, 80, 96, 130)),
                BackMaterial = MaterialHelper.CreateMaterial(Color.FromArgb(80, 80, 96, 130)),
            });
        }

        private void AddCellCloud(Model3DGroup group, IReadOnlyList<TraceCell> cells, Color color, double scale, double alpha)
        {
            if (cells.Count == 0) return;
            var mb = new MeshBuilder(false, false);
            double size = Math.Max(_cellMm * scale, 6.0);
            foreach (var c in cells)
                mb.AddBox(CellCenter(c.I, c.J, c.K), size, size, size);

            byte a = (byte)Math.Clamp((int)(alpha * 255), 20, 255);
            var material = MaterialHelper.CreateMaterial(Color.FromArgb(a, color.R, color.G, color.B));
            group.Children.Add(new GeometryModel3D
            {
                Geometry = mb.ToMesh(true),
                Material = material,
                BackMaterial = material,
            });
        }

        private void AddCellsForEvent(Model3DGroup group, JsonElement root, List<Point3D> focusPoints)
        {
            var type = GetString(root, "type") ?? "";
            if (type == "task_begin")
            {
                AddCellIfPresent(group, root, "source_cell", Colors.LimeGreen, 3.0, focusPoints);
                AddCellIfPresent(group, root, "target_cell", Colors.Red, 3.0, focusPoints);
                AddCellIfPresent(group, root, "snapped_source", Colors.DeepSkyBlue, 2.4, focusPoints);
                AddCellIfPresent(group, root, "snapped_target", Colors.Yellow, 2.4, focusPoints);
            }
            else if (type == "snap")
            {
                AddCellIfPresent(group, root, "from", Colors.Orange, 2.8, focusPoints);
                AddCellIfPresent(group, root, "to", Colors.DeepSkyBlue, 2.8, focusPoints);
            }
            else if (type == "expand_cell")
            {
                AddCellIfPresent(group, root, "cell", Colors.Gold, 3.0, focusPoints);
            }
            else if (type == "candidate_reject")
            {
                AddCellIfPresent(group, root, "from", Colors.DeepSkyBlue, 2.4, focusPoints);
                AddCellIfPresent(group, root, "to", Colors.OrangeRed, 3.2, focusPoints);
            }
        }

        private void AddCellIfPresent(Model3DGroup group, JsonElement root, string name, Color color, double scale, List<Point3D> focusPoints)
        {
            if (!TryGetArray(root, name, out var arr) || arr.GetArrayLength() < 3) return;
            int i = arr[0].GetInt32();
            int j = arr[1].GetInt32();
            int k = arr[2].GetInt32();
            var center = CellCenter(i, j, k);
            focusPoints.Add(center);
            var mb = new MeshBuilder(false, false);
            double size = Math.Max(_cellMm * scale, 70.0);
            mb.AddBox(center, size, size, size);
            var material = MaterialHelper.CreateMaterial(Color.FromArgb(240, color.R, color.G, color.B));
            group.Children.Add(new GeometryModel3D
            {
                Geometry = mb.ToMesh(true),
                Material = material,
                BackMaterial = material,
            });
        }

        private Point3D CellCenter(int i, int j, int k)
            => new(_ox + (i + 0.5) * _cellMm, _oy + (j + 0.5) * _cellMm, _oz + (k + 0.5) * _cellMm);

        private static void AddBoxFrame(Model3DGroup group, Point3D lo, Point3D hi, Color color, double r)
        {
            var pts = new[]
            {
                new Point3D(lo.X, lo.Y, lo.Z), new Point3D(hi.X, lo.Y, lo.Z),
                new Point3D(hi.X, hi.Y, lo.Z), new Point3D(lo.X, hi.Y, lo.Z),
                new Point3D(lo.X, lo.Y, hi.Z), new Point3D(hi.X, lo.Y, hi.Z),
                new Point3D(hi.X, hi.Y, hi.Z), new Point3D(lo.X, hi.Y, hi.Z),
            };
            int[,] edges =
            {
                {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
            };
            var mb = new MeshBuilder(false, false);
            for (int e = 0; e < 12; e++) mb.AddCylinder(pts[edges[e, 0]], pts[edges[e, 1]], r, 6);
            group.Children.Add(new GeometryModel3D
            {
                Geometry = mb.ToMesh(true),
                Material = MaterialHelper.CreateMaterial(color),
                BackMaterial = MaterialHelper.CreateMaterial(color),
            });
        }

        private void OnFirst(object sender, RoutedEventArgs e) => SelectVisibleIndex(0);
        private void OnPrev(object sender, RoutedEventArgs e) => SelectVisibleOffset(-1);
        private void OnNext(object sender, RoutedEventArgs e) => SelectVisibleOffset(1);
        private void OnLast(object sender, RoutedEventArgs e) => SelectVisibleIndex(VisibleRows().Count - 1);
        private void OnFit(object sender, RoutedEventArgs e)
        {
            if (!FocusCurrentEvent()) TraceView.ZoomExtents();
        }

        private Rect3D? BuildFocusBounds(IReadOnlyList<Point3D> points)
        {
            if (points.Count == 0) return null;
            double pad = Math.Max(_cellMm * 12.0, 300.0);
            double minX = points.Min(p => p.X) - pad, minY = points.Min(p => p.Y) - pad, minZ = points.Min(p => p.Z) - pad;
            double maxX = points.Max(p => p.X) + pad, maxY = points.Max(p => p.Y) + pad, maxZ = points.Max(p => p.Z) + pad;
            return new Rect3D(minX, minY, minZ, maxX - minX, maxY - minY, maxZ - minZ);
        }

        private bool FocusCurrentEvent()
        {
            if (_lastFocusBounds is not { } b || b.IsEmpty) return false;
            var center = new Point3D(b.X + b.SizeX * 0.5, b.Y + b.SizeY * 0.5, b.Z + b.SizeZ * 0.5);
            double diag = Math.Sqrt(b.SizeX * b.SizeX + b.SizeY * b.SizeY + b.SizeZ * b.SizeZ);
            double dist = Math.Max(diag * 1.7, _cellMm * 80.0);
            var dir = new Vector3D(-1.2, -1.4, -0.85);
            dir.Normalize();

            if (TraceView.Camera is ProjectionCamera camera)
            {
                camera.Position = center - dir * dist;
                camera.LookDirection = dir * dist;
                camera.UpDirection = new Vector3D(0, 0, 1);
                camera.NearPlaneDistance = Math.Max(0.1, dist / 1000.0);
                camera.FarPlaneDistance = Math.Max(dist * 20.0, diag * 20.0);
                return true;
            }

            return false;
        }

        private RenderTargetBitmap? CaptureViewportBitmap()
        {
            TraceView.UpdateLayout();
            int width = Math.Max(1, (int)Math.Round(TraceView.ActualWidth));
            int height = Math.Max(1, (int)Math.Round(TraceView.ActualHeight));
            if (width <= 1 || height <= 1) return null;

            var bitmap = new RenderTargetBitmap(width, height, 96, 96, PixelFormats.Pbgra32);
            bitmap.Render(TraceView);
            return bitmap;
        }

        private void OnCopyImage(object sender, RoutedEventArgs e)
        {
            var bitmap = CaptureViewportBitmap();
            if (bitmap == null) return;
            Clipboard.SetImage(bitmap);
            StatusText.Text = "Current 3D view copied to clipboard.";
        }

        private void OnSaveImage(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Title = "Save current trace view image",
                Filter = "PNG image (*.png)|*.png",
                FileName = $"routing_trace_view_{DateTime.Now:yyyyMMdd_HHmmss}.png",
            };
            if (dlg.ShowDialog(this) != true) return;

            var bitmap = CaptureViewportBitmap();
            if (bitmap == null) return;
            SavePng(bitmap, dlg.FileName);
            StatusText.Text = $"Image saved: {dlg.FileName}";
        }

        private void OnSaveVideo(object sender, RoutedEventArgs e)
        {
            if (_savingVideo) return;
            var rows = VisibleRows();
            if (rows.Count == 0) return;

            var dlg = new SaveFileDialog
            {
                Title = "Save trace replay video",
                Filter = "MP4 video (*.mp4)|*.mp4",
                FileName = $"routing_trace_replay_{DateTime.Now:yyyyMMdd_HHmmss}.mp4",
            };
            if (dlg.ShowDialog(this) != true) return;

            _savingVideo = true;
            StopPlayback();
            var selected = EventGrid.SelectedItem as TraceEventRow;
            string frameDir = Path.Combine(Path.GetDirectoryName(dlg.FileName) ?? AppContext.BaseDirectory,
                                           Path.GetFileNameWithoutExtension(dlg.FileName) + "_frames");
            Directory.CreateDirectory(frameDir);

            try
            {
                for (int i = 0; i < rows.Count; i++)
                {
                    SelectVisibleIndex(i);
                    Dispatcher.Invoke(() => { }, DispatcherPriority.Render);
                    var bitmap = CaptureViewportBitmap();
                    if (bitmap == null) continue;
                    SavePng(bitmap, Path.Combine(frameDir, $"frame_{i:00000}.png"));
                    if (i % 20 == 0)
                        StatusText.Text = $"Saving video frames... {i + 1:N0}/{rows.Count:N0}";
                }

                bool encoded = TryEncodeVideoWithFfmpeg(frameDir, dlg.FileName);
                StatusText.Text = encoded
                    ? $"Video saved: {dlg.FileName}"
                    : $"Frames saved: {frameDir} (ffmpeg not found or failed)";
            }
            finally
            {
                _savingVideo = false;
                if (selected != null)
                {
                    EventGrid.SelectedItem = selected;
                    EventGrid.ScrollIntoView(selected);
                }
            }
        }

        private static void SavePng(BitmapSource bitmap, string path)
        {
            var encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));
            using var fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.Read);
            encoder.Save(fs);
        }

        private bool TryEncodeVideoWithFfmpeg(string frameDir, string outputPath)
        {
            try
            {
                string input = Path.Combine(frameDir, "frame_%05d.png");
                var psi = new ProcessStartInfo
                {
                    FileName = "ffmpeg",
                    Arguments = $"-y -framerate 10 -i \"{input}\" -pix_fmt yuv420p \"{outputPath}\"",
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardError = true,
                    RedirectStandardOutput = true,
                };
                using var p = Process.Start(psi);
                if (p == null) return false;
                p.WaitForExit();
                return p.ExitCode == 0 && File.Exists(outputPath);
            }
            catch
            {
                return false;
            }
        }

        private void OnPlayPause(object sender, RoutedEventArgs e)
        {
            if (_isPlaying) StopPlayback();
            else StartPlayback();
        }

        private void OnSpeedChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (SpeedText == null || _playTimer == null) return;
            int ms = Math.Max(10, (int)e.NewValue);
            SpeedText.Text = ms.ToString();
            _playTimer.Interval = TimeSpan.FromMilliseconds(ms);
        }

        private void OnPlaybackTick(object? sender, EventArgs e)
        {
            if (PathPlaybackBox?.IsChecked == true)
            {
                AdvancePathPlayback();
                return;
            }

            var rows = VisibleRows();
            if (rows.Count == 0)
            {
                StopPlayback();
                return;
            }

            var selected = EventGrid.SelectedItem as TraceEventRow;
            int current = selected == null ? -1 : rows.IndexOf(selected);
            if (current < 0)
            {
                SelectVisibleIndex(0);
                return;
            }

            if (current >= rows.Count - 1)
            {
                StopPlayback();
                return;
            }

            SelectVisibleIndex(current + 1);
        }

        private bool PreparePathPlayback()
        {
            var selected = EventGrid.SelectedItem as TraceEventRow;
            var row = selected?.Type == "route_path" ? selected : FindRoutePathRow(selected);
            if (row == null)
            {
                StatusText.Text = "Path Playback needs a route_path event. Select a route_path row or a routed task.";
                return false;
            }

            EventGrid.SelectedItem = row;
            EventGrid.ScrollIntoView(row);
            var cells = ReadRoutePathCells(row);
            if (cells.Count < 2)
            {
                StatusText.Text = "Selected route_path has no drawable path cells.";
                return false;
            }

            _pathPlaybackCells = 1;
            RebuildModel(row);
            StatusText.Text = $"Path Playback 1/{cells.Count:N0} cells | task={row.TaskText}";
            return true;
        }

        private void AdvancePathPlayback()
        {
            var row = EventGrid.SelectedItem as TraceEventRow;
            if (row?.Type != "route_path")
            {
                if (!PreparePathPlayback()) StopPlayback();
                return;
            }

            var cells = ReadRoutePathCells(row);
            if (cells.Count < 2)
            {
                StopPlayback();
                return;
            }

            if (_pathPlaybackCells >= cells.Count)
            {
                StopPlayback();
                StatusText.Text = $"Path Playback complete {cells.Count:N0}/{cells.Count:N0} cells | task={row.TaskText}";
                return;
            }

            _pathPlaybackCells = Math.Clamp(_pathPlaybackCells + 1, 1, cells.Count);
            RebuildModel(row);
            StatusText.Text = $"Path Playback {_pathPlaybackCells:N0}/{cells.Count:N0} cells | task={row.TaskText}";
        }

        private TraceEventRow? FindRoutePathRow(TraceEventRow? selected)
        {
            if (selected?.Task is int task)
            {
                var byTask = Events.FirstOrDefault(e => e.Task == task && e.Type == "route_path");
                if (byTask != null) return byTask;
            }
            return VisibleRows().FirstOrDefault(e => e.Type == "route_path")
                ?? Events.FirstOrDefault(e => e.Type == "route_path");
        }

        private static List<TraceCell> ReadRoutePathCells(TraceEventRow row)
        {
            try
            {
                using var doc = JsonDocument.Parse(row.RawJson);
                return TryGetArray(doc.RootElement, "cells", out var pathCells)
                    ? ReadCells(pathCells).ToList()
                    : new List<TraceCell>();
            }
            catch
            {
                return new List<TraceCell>();
            }
        }
        private void StartPlayback()
        {
            if (PathPlaybackBox?.IsChecked == true)
            {
                if (!PreparePathPlayback()) return;
            }
            else if (VisibleRows().Count == 0) return;
            _isPlaying = true;
            PlayButton.Content = "Pause";
            _playTimer.Start();
        }

        private void StopPlayback()
        {
            _isPlaying = false;
            if (PlayButton != null) PlayButton.Content = "Play";
            _playTimer?.Stop();
        }

        private void SelectVisibleOffset(int offset)
        {
            var rows = VisibleRows();
            if (rows.Count == 0) return;
            var selected = EventGrid.SelectedItem as TraceEventRow;
            int current = selected == null ? -1 : rows.IndexOf(selected);
            if (current < 0) current = 0;
            SelectVisibleIndex(current + offset);
        }

        private void SelectVisibleIndex(int idx)
        {
            var rows = VisibleRows();
            if (rows.Count == 0)
            {
                EventGrid.SelectedItem = null;
                DetailText.Text = "";
                RebuildModel(null);
                return;
            }
            idx = Math.Clamp(idx, 0, rows.Count - 1);
            EventGrid.SelectedItem = rows[idx];
            EventGrid.ScrollIntoView(rows[idx]);
        }

        private void SelectFirstVisible() => SelectVisibleIndex(0);

        private List<TraceEventRow> VisibleRows()
            => _eventsView.Cast<TraceEventRow>().ToList();

        private static string Summarize(JsonElement root)
        {
            var type = GetString(root, "type") ?? "";
            return type switch
            {
                "trace_header" => $"cell={GetDoubleText(root, "cell_mm")} shape={ArrayText(root, "shape")}",
                "effective_options" => $"segment={GetBoolText(root, "segment_astar")} octree={GetBoolText(root, "octree_guide")} split={GetBoolText(root, "route_split")} maxExp={GetLongText(root, "max_expansions")}",
                "occupancy_summary" => $"blocked={GetLongText(root, "blocked_count")}",
                "occupancy_sample" => $"sampled={GetLongText(root, "sampled")}/{GetLongText(root, "total")}",
                "passthrough_sample" => $"sampled={GetLongText(root, "sampled")}/{GetLongText(root, "total")}",
                "task_begin" => $"src={ArrayText(root, "source_cell")} dst={ArrayText(root, "target_cell")} snap={ArrayText(root, "snapped_source")}->{ArrayText(root, "snapped_target")}",
                "snap" => $"{GetString(root, "kind")} {ArrayText(root, "from")} -> {ArrayText(root, "to")}",
                "expand" => $"expanded={GetLongText(root, "expanded_nodes")} progress={GetDoubleText(root, "progress01")}",
                "expand_cell" => $"cell={ArrayText(root, "cell")} exp={GetLongText(root, "expanded_nodes")} dir={GetLongText(root, "dir")} run={GetLongText(root, "run")}",
                "candidate_reject" => $"{GetString(root, "reason")} {ArrayText(root, "from")} -> {ArrayText(root, "to")} exp={GetLongText(root, "expanded_nodes")} run={GetLongText(root, "run")}/{GetLongText(root, "required")}",
                "task_end" => $"success={GetBoolText(root, "success")} len={GetDoubleText(root, "length_mm")} turns={GetLongText(root, "turns")} exp={GetLongText(root, "expanded_nodes")}",
                "postprocess" => $"{GetString(root, "stage")} turns {GetLongText(root, "before_turns")}->{GetLongText(root, "after_turns")} points {GetLongText(root, "before_points")}->{GetLongText(root, "after_points")}",
                "route_split_plan" => $"trunkK={GetLongText(root, "trunk_k")} z={GetDoubleText(root, "trunk_z_mm")} source={GetString(root, "source")}",
                "route_split_segment" => $"seg={GetLongText(root, "segment")} {ArrayText(root, "from")}->{ArrayText(root, "to")} ok={GetBoolText(root, "success")} exp={GetLongText(root, "expanded_nodes")}",
                "route_mark" => $"path={GetLongText(root, "path_points")} radius={GetLongText(root, "radius_cells")}",
                "route_path" => $"path cells={GetLongText(root, "path_points")}",
                "trace_limit" => $"task log limit reached max={GetLongText(root, "max_events")}",
                _ => root.ToString()
            };
        }

        private static string? GetString(JsonElement e, string name)
            => e.TryGetProperty(name, out var p) && p.ValueKind == JsonValueKind.String ? p.GetString() : null;

        private static int? GetInt(JsonElement e, string name)
            => e.TryGetProperty(name, out var p) && p.ValueKind == JsonValueKind.Number && p.TryGetInt32(out var v) ? v : null;

        private static bool TryGetDouble(JsonElement e, string name, out double value)
        {
            value = 0;
            return e.TryGetProperty(name, out var p) && p.ValueKind == JsonValueKind.Number && p.TryGetDouble(out value);
        }

        private static bool TryGetArray(JsonElement e, string name, out JsonElement arr)
        {
            arr = default;
            return e.TryGetProperty(name, out arr) && arr.ValueKind == JsonValueKind.Array;
        }

        private static bool TryGetCell(JsonElement e, string name, out TraceCell cell)
        {
            cell = default;
            if (!TryGetArray(e, name, out var arr) || arr.GetArrayLength() < 3) return false;
            cell = new TraceCell(arr[0].GetInt32(), arr[1].GetInt32(), arr[2].GetInt32());
            return true;
        }

        private static string ArrayText(JsonElement e, string name)
            => TryGetArray(e, name, out var arr) ? arr.ToString() : "-";

        private static string GetLongText(JsonElement e, string name)
            => e.TryGetProperty(name, out var p) ? p.ToString() : "-";

        private static string GetDoubleText(JsonElement e, string name)
            => e.TryGetProperty(name, out var p) && p.ValueKind == JsonValueKind.Number && p.TryGetDouble(out var v)
                ? v.ToString("0.###")
                : "-";

        private static string GetBoolText(JsonElement e, string name)
            => e.TryGetProperty(name, out var p) ? p.ToString() : "-";
    }

    public sealed record TraceEventRow(int Seq, string Type, int? Task, string Summary, string RawJson)
    {
        public string TaskText => Task.HasValue ? Task.Value.ToString() : "";
    }

    public readonly record struct TraceCell(int I, int J, int K);
    public readonly record struct CellBounds(int MinI, int MaxI, int MinJ, int MaxJ, int MinK, int MaxK);
}
