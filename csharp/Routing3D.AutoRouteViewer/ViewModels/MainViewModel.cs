using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using Routing3D.AutoRouteViewer.Models;
using Routing3D.AutoRouteViewer.Services;

namespace Routing3D.AutoRouteViewer.ViewModels;

public sealed class MainViewModel : INotifyPropertyChanged
{
    private readonly DesignDataLoader _loader = new();
    private readonly AutoRouteModuleRunner _runner = new();
    private readonly List<RoutePath> _allRoutes = new();
    private SceneSnapshot _scene = new();
    private CancellationTokenSource? _cts;
    private ProjectOption? _selectedProject;
    private RoutePath? _selectedRoute;
    private RouteComparison? _selectedComparison;
    private string? _selectedUtilityGroup;
    private string? _selectedUtility;
    private Model3DGroup _sceneModel = new();
    private string _status = "Ready";
    private bool _isBusy;
    private bool _suppressProjectAutoLoad;
    private bool _suppressFilterEvents;
    private bool _showSpaces = true;
    private bool _showObstacles = true;
    private bool _showEquipment = true;
    private bool _showDucts = true;
    private bool _showLaterals = true;
    private bool _showExistingPipes = true;
    private bool _showAutoRoutes = true;
    private bool _showPocMarkers = true;
    private bool _showFittings = true;
    private bool _showBoundsFrame = true;

    public MainViewModel()
    {
        Host = Environment.GetEnvironmentVariable("PGHOST") ?? "localhost";
        Port = int.TryParse(Environment.GetEnvironmentVariable("PGPORT"), out int p) ? p : 5432;
        User = Environment.GetEnvironmentVariable("PGUSER") ?? "postgres";
        Password = Environment.GetEnvironmentVariable("PGPASSWORD") ?? "dinno";
        Database = Environment.GetEnvironmentVariable("PGDATABASE") ?? "DDW_AI_DB";
        RouteSql = string.Empty;
        ObstacleSql = string.Empty;
        VoxelSize = 25;
        MaxSearchNodes = 100000;
        TimeoutSeconds = 30;
        UseSweepGeometry = true;
        LoadDemoCommand = new AsyncCommand(LoadDemoAsync, () => !IsBusy);
        LoadProjectsCommand = new AsyncCommand(LoadProjectsAsync, () => !IsBusy);
        LoadPostgresCommand = new AsyncCommand(LoadPostgresAsync, () => !IsBusy && SelectedProject != null);
        RouteSelectedCommand = new AsyncCommand(RouteSelectedAsync, () => !IsBusy && SelectedRoute != null);
        RouteFilteredCommand = new AsyncCommand(RouteFilteredAsync, () => !IsBusy && Routes.Count > 0);
        RouteAllCommand = new AsyncCommand(RouteAllAsync, () => !IsBusy && _allRoutes.Count > 0);
        CancelCommand = new RelayCommand(() => _cts?.Cancel(), () => IsBusy);
        _ = LoadDemoAsync();
    }

    public ObservableCollection<ProjectOption> Projects { get; } = new();
    public ObservableCollection<RoutePath> Routes { get; } = new();
    public ObservableCollection<ObstacleBox> Obstacles { get; } = new();
    public ObservableCollection<RouteComparison> Comparisons { get; } = new();
    public ObservableCollection<string> UtilityGroups { get; } = new();
    public ObservableCollection<string> Utilities { get; } = new();

    public string Host { get; set; }
    public int Port { get; set; }
    public string User { get; set; }
    public string Password { get; set; }
    public string Database { get; set; }
    public string RouteSql { get; set; }
    public string ObstacleSql { get; set; }
    public double VoxelSize { get; set; }
    public int MaxSearchNodes { get; set; }
    public int TimeoutSeconds { get; set; }
    public bool UseSweepGeometry { get; set; }

    public bool ShowSpaces { get => _showSpaces; set => SetLayer(ref _showSpaces, value); }
    public bool ShowObstacles { get => _showObstacles; set => SetLayer(ref _showObstacles, value); }
    public bool ShowEquipment { get => _showEquipment; set => SetLayer(ref _showEquipment, value); }
    public bool ShowDucts { get => _showDucts; set => SetLayer(ref _showDucts, value); }
    public bool ShowLaterals { get => _showLaterals; set => SetLayer(ref _showLaterals, value); }
    public bool ShowExistingPipes { get => _showExistingPipes; set => SetLayer(ref _showExistingPipes, value); }
    public bool ShowAutoRoutes { get => _showAutoRoutes; set => SetLayer(ref _showAutoRoutes, value); }
    public bool ShowPocMarkers { get => _showPocMarkers; set => SetLayer(ref _showPocMarkers, value); }
    public bool ShowFittings { get => _showFittings; set => SetLayer(ref _showFittings, value); }
    public bool ShowBoundsFrame { get => _showBoundsFrame; set => SetLayer(ref _showBoundsFrame, value); }

    public int ExistingPipeCount => _scene.ExistingPipes.Count;
    public int EquipmentCount => _scene.Equipment.Count;
    public int DuctCount => _scene.Ducts.Count;
    public int LateralCount => _scene.Laterals.Count;
    public int PocCount => _scene.Pocs.Count;
    public int FittingCount => _scene.Fittings.Count;
    public int RoutingSolidCount => _scene.RoutingSolids.Count();

    public string ConnectionString => $"Host={Host};Port={Port};Database={Database};Username={User};Password={Password};Timeout=10;Encoding=UTF8";

    public ProjectOption? SelectedProject
    {
        get => _selectedProject;
        set
        {
            if (Equals(_selectedProject, value)) return;
            _selectedProject = value;
            OnPropertyChanged();
            CommandManager.InvalidateRequerySuggested();
            if (!_suppressProjectAutoLoad && value != null && !IsBusy)
                _ = LoadPostgresAsync();
        }
    }

    public string? SelectedUtilityGroup
    {
        get => _selectedUtilityGroup;
        set
        {
            if (_selectedUtilityGroup == value) return;
            _selectedUtilityGroup = value;
            OnPropertyChanged();
            if (!_suppressFilterEvents)
            {
                RebuildUtilitiesForSelectedGroup();
                ApplyFilters();
            }
        }
    }

    public string? SelectedUtility
    {
        get => _selectedUtility;
        set
        {
            if (_selectedUtility == value) return;
            _selectedUtility = value;
            OnPropertyChanged();
            if (!_suppressFilterEvents) ApplyFilters();
        }
    }

    public RoutePath? SelectedRoute
    {
        get => _selectedRoute;
        set { _selectedRoute = value; OnPropertyChanged(); RefreshScene(); CommandManager.InvalidateRequerySuggested(); }
    }

    public RouteComparison? SelectedComparison
    {
        get => _selectedComparison;
        set { _selectedComparison = value; OnPropertyChanged(); RefreshScene(); }
    }

    public Model3DGroup SceneModel { get => _sceneModel; private set { _sceneModel = value; OnPropertyChanged(); } }
    public string Status { get => _status; private set { _status = value; OnPropertyChanged(); } }
    public bool IsBusy { get => _isBusy; private set { _isBusy = value; OnPropertyChanged(); CommandManager.InvalidateRequerySuggested(); } }

    public ICommand LoadDemoCommand { get; }
    public ICommand LoadProjectsCommand { get; }
    public ICommand LoadPostgresCommand { get; }
    public ICommand RouteSelectedCommand { get; }
    public ICommand RouteFilteredCommand { get; }
    public ICommand RouteAllCommand { get; }
    public ICommand CancelCommand { get; }

    public event PropertyChangedEventHandler? PropertyChanged;

    private Task LoadDemoAsync()
    {
        ReplaceData(_loader.LoadDemo());
        Status = BuildLoadStatus("Demo loaded");
        return Task.CompletedTask;
    }

    private async Task LoadProjectsAsync()
    {
        await RunBusyAsync(async token =>
        {
            Status = "Loading projects from PostgreSQL...";
            List<ProjectOption> projects = await _loader.ListProjectsAsync(ConnectionString, token);
            _suppressProjectAutoLoad = true;
            Projects.Clear();
            foreach (ProjectOption project in projects) Projects.Add(project);
            SelectedProject = Projects.FirstOrDefault();
            _suppressProjectAutoLoad = false;
            Status = $"Projects loaded: {Projects.Count}";
            if (SelectedProject != null)
                await LoadSelectedProjectCoreAsync(token);
        });
    }

    private async Task LoadPostgresAsync()
    {
        if (SelectedProject == null) return;
        await RunBusyAsync(LoadSelectedProjectCoreAsync);
    }

    private async Task LoadSelectedProjectCoreAsync(CancellationToken token)
    {
        if (SelectedProject == null) return;
        Status = $"Loading {SelectedProject.Display}: equipment, ducts, obstacles, PoCs, existing routes...";
        SceneSnapshot scene = await _loader.LoadProjectAsync(ConnectionString, SelectedProject, RouteSql, ObstacleSql, token);
        ReplaceData(scene);
        Status = BuildLoadStatus($"Loaded {SelectedProject.Display}");
    }

    private async Task RouteSelectedAsync()
    {
        if (SelectedRoute == null) return;
        await RouteManyAsync(new[] { SelectedRoute });
    }

    private async Task RouteFilteredAsync() => await RouteManyAsync(Routes.ToList());
    private async Task RouteAllAsync() => await RouteManyAsync(_allRoutes.ToList());

    private async Task RouteManyAsync(IReadOnlyList<RoutePath> targets)
    {
        await RunBusyAsync(async token =>
        {
            if (targets.Count != 1) Comparisons.Clear();
            int done = 0;
            var solids = _scene.RoutingSolids.ToList();
            foreach (RoutePath route in targets)
            {
                token.ThrowIfCancellationRequested();
                Status = $"Routing {++done}/{targets.Count}: {route.RouteId} · solids={solids.Count}";
                RouteComparison comparison = await _runner.RouteAsync(route, solids, (float)VoxelSize, MaxSearchNodes, TimeSpan.FromSeconds(Math.Max(1, TimeoutSeconds)), UseSweepGeometry, token);
                UpsertComparison(comparison);
                SelectedComparison = comparison;
            }
            Status = $"Routing completed: {targets.Count} route(s).";
        });
    }

    private async Task RunBusyAsync(Func<CancellationToken, Task> action)
    {
        if (IsBusy) return;
        _cts = new CancellationTokenSource();
        IsBusy = true;
        try { await action(_cts.Token); }
        catch (OperationCanceledException) { Status = "Cancelled."; }
        catch (Exception ex) { Status = ex.Message; }
        finally { IsBusy = false; _cts.Dispose(); _cts = null; }
    }

    private void ReplaceData(SceneSnapshot scene)
    {
        _scene = scene;
        _allRoutes.Clear(); _allRoutes.AddRange(scene.Routes);
        Obstacles.Clear(); foreach (ObstacleBox obstacle in scene.Obstacles) Obstacles.Add(obstacle);
        RebuildFilters();
        ApplyFilters();
        Comparisons.Clear();
        SelectedComparison = null;
        NotifySceneCounts();
        RefreshScene();
    }

    private void RebuildFilters()
    {
        _suppressFilterEvents = true;
        UtilityGroups.Clear(); UtilityGroups.Add("(All)");
        foreach (string v in _allRoutes.Select(r => NormalizeFilterValue(r.UtilityGroup)).Distinct().OrderBy(x => x)) UtilityGroups.Add(v);
        _selectedUtilityGroup = UtilityGroups.FirstOrDefault(); OnPropertyChanged(nameof(SelectedUtilityGroup));
        RebuildUtilitiesForSelectedGroup();
        _suppressFilterEvents = false;
    }

    private void RebuildUtilitiesForSelectedGroup()
    {
        string? current = _selectedUtility;
        string? group = _selectedUtilityGroup;
        Utilities.Clear();
        Utilities.Add("(All)");
        foreach (string v in _allRoutes
            .Where(r => string.IsNullOrEmpty(group) || group == "(All)" || string.Equals(NormalizeFilterValue(r.UtilityGroup), group, StringComparison.OrdinalIgnoreCase))
            .Select(r => NormalizeFilterValue(r.Utility))
            .Distinct()
            .OrderBy(x => x))
        {
            Utilities.Add(v);
        }
        _selectedUtility = current != null && Utilities.Contains(current) ? current : Utilities.FirstOrDefault();
        OnPropertyChanged(nameof(SelectedUtility));
    }

    private static string NormalizeFilterValue(string? value) => string.IsNullOrWhiteSpace(value) ? "(None)" : value;
    private void ApplyFilters()
    {
        string? group = SelectedUtilityGroup;
        string? util = SelectedUtility;
        Routes.Clear();
        foreach (RoutePath r in _allRoutes)
        {
            bool groupOk = string.IsNullOrEmpty(group) || group == "(All)" || string.Equals(NormalizeFilterValue(r.UtilityGroup), group, StringComparison.OrdinalIgnoreCase);
            bool utilOk = string.IsNullOrEmpty(util) || util == "(All)" || string.Equals(NormalizeFilterValue(r.Utility), util, StringComparison.OrdinalIgnoreCase);
            if (groupOk && utilOk) Routes.Add(r);
        }
        SelectedRoute = Routes.FirstOrDefault();
        Status = $"Filtered routes: {Routes.Count}/{_allRoutes.Count}";
        RefreshScene();
    }

    private bool RouteMatchesCurrentFilter(RoutePath r)
    {
        string? group = SelectedUtilityGroup;
        string? util = SelectedUtility;
        bool groupOk = string.IsNullOrEmpty(group) || group == "(All)" || string.Equals(NormalizeFilterValue(r.UtilityGroup), group, StringComparison.OrdinalIgnoreCase);
        bool utilOk = string.IsNullOrEmpty(util) || util == "(All)" || string.Equals(NormalizeFilterValue(r.Utility), util, StringComparison.OrdinalIgnoreCase);
        return groupOk && utilOk;
    }

    private List<RoutePath> FilteredExistingPipes() => _scene.ExistingPipes.Where(RouteMatchesCurrentFilter).ToList();

    private SceneRenderOptions RenderOptions() => new()
    {
        ShowSpaces = ShowSpaces,
        ShowObstacles = ShowObstacles,
        ShowEquipment = ShowEquipment,
        ShowDucts = ShowDucts,
        ShowLaterals = ShowLaterals,
        ShowExistingPipes = ShowExistingPipes,
        ShowAutoRoutes = ShowAutoRoutes,
        ShowPocMarkers = ShowPocMarkers,
        ShowFittings = ShowFittings,
        ShowBoundsFrame = ShowBoundsFrame
    };

    private void SetLayer(ref bool field, bool value, [CallerMemberName] string? name = null)
    {
        if (field == value) return;
        field = value;
        OnPropertyChanged(name);
        RefreshScene();
    }

    private void UpsertComparison(RouteComparison comparison)
    {
        for (int i = 0; i < Comparisons.Count; i++)
        {
            if (Comparisons[i].RouteId == comparison.RouteId) { Comparisons[i] = comparison; return; }
        }
        Comparisons.Add(comparison);
    }

    private string BuildLoadStatus(string prefix) =>
        $"{prefix}: routes={_allRoutes.Count}, existing pipes={_scene.ExistingPipes.Count}, obstacles={_scene.Obstacles.Count}, equipment={_scene.Equipment.Count}, ducts={_scene.Ducts.Count}, laterals={_scene.Laterals.Count}, PoCs={_scene.Pocs.Count}, fittings={_scene.Fittings.Count}, routing solids={RoutingSolidCount}";

    private void NotifySceneCounts()
    {
        OnPropertyChanged(nameof(ExistingPipeCount));
        OnPropertyChanged(nameof(EquipmentCount));
        OnPropertyChanged(nameof(DuctCount));
        OnPropertyChanged(nameof(LateralCount));
        OnPropertyChanged(nameof(PocCount));
        OnPropertyChanged(nameof(FittingCount));
        OnPropertyChanged(nameof(RoutingSolidCount));
    }

    private void RefreshScene() => SceneModel = SceneModelBuilder.Build(_scene, Routes, FilteredExistingPipes(), SelectedComparison, RenderOptions());
    private void OnPropertyChanged([CallerMemberName] string? name = null) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}

public sealed class RelayCommand : ICommand
{
    private readonly Action _execute;
    private readonly Func<bool>? _canExecute;
    public RelayCommand(Action execute, Func<bool>? canExecute = null) { _execute = execute; _canExecute = canExecute; }
    public bool CanExecute(object? parameter) => _canExecute?.Invoke() ?? true;
    public void Execute(object? parameter) => _execute();
    public event EventHandler? CanExecuteChanged { add => CommandManager.RequerySuggested += value; remove => CommandManager.RequerySuggested -= value; }
}

public sealed class AsyncCommand : ICommand
{
    private readonly Func<Task> _execute;
    private readonly Func<bool>? _canExecute;
    public AsyncCommand(Func<Task> execute, Func<bool>? canExecute = null) { _execute = execute; _canExecute = canExecute; }
    public bool CanExecute(object? parameter) => _canExecute?.Invoke() ?? true;
    public async void Execute(object? parameter) => await _execute();
    public event EventHandler? CanExecuteChanged { add => CommandManager.RequerySuggested += value; remove => CommandManager.RequerySuggested -= value; }
}
