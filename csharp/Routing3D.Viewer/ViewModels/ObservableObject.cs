using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Routing3D.Viewer.ViewModels
{
    /// <summary>INotifyPropertyChanged 보일러플레이트를 줄이는 기반 클래스.</summary>
    public abstract class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        protected void OnChanged([CallerMemberName] string? name = null) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

        protected bool Set<T>(ref T field, T value, [CallerMemberName] string? name = null)
        {
            if (EqualityComparer<T>.Default.Equals(field, value)) return false;
            field = value;
            OnChanged(name);
            return true;
        }
    }
}
