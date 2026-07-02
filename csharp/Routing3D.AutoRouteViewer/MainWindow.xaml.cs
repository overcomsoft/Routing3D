using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Threading;
using Routing3D.AutoRouteViewer.ViewModels;

namespace Routing3D.AutoRouteViewer;

public partial class MainWindow : Window
{
    private MainViewModel? _viewModel;

    public MainWindow()
    {
        InitializeComponent();
        Loaded += OnLoaded;
        DataContextChanged += OnDataContextChanged;
        AttachViewModel(DataContext as MainViewModel);
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        FitViewSoon();
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        AttachViewModel(e.NewValue as MainViewModel);
    }

    private void AttachViewModel(MainViewModel? viewModel)
    {
        if (ReferenceEquals(_viewModel, viewModel)) return;
        if (_viewModel != null) _viewModel.PropertyChanged -= OnViewModelPropertyChanged;
        _viewModel = viewModel;
        if (_viewModel != null) _viewModel.PropertyChanged += OnViewModelPropertyChanged;
    }

    private void OnViewModelPropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName == nameof(MainViewModel.SceneModel))
            FitViewSoon();
    }

    private void FitView_Click(object sender, RoutedEventArgs e)
    {
        FitViewSoon();
    }

    private void FitViewSoon()
    {
        Dispatcher.BeginInvoke(new Action(() =>
        {
            try
            {
                MainViewport.ZoomExtents(500);
            }
            catch
            {
                // The viewport can be temporarily unavailable during startup/layout.
            }
        }), DispatcherPriority.ContextIdle);
    }
}
