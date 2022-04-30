using System.Windows.Controls;

using Demo.ViewModels.Interfaces;
using ModernWpf.Controls;
using Prism.Commands;

namespace Demo.DesignTimes
{

    internal sealed class MainWindowViewModel : IMainWindowViewModel
    {

        #region IMainWindowViewModel Membbers

        public DelegateCommand<SelectionChangedEventArgs> PageListSelectionChanged
        {
            get;
        }

        public DelegateCommand<AutoSuggestBoxQuerySubmittedEventArgs> QuerySubmittedCommand
        {
            get;
        }

        public DelegateCommand<AutoSuggestBoxTextChangedEventArgs> QueryTextChangedCommand
        {
            get;
        }

        public DelegateCommand ThemeChangeCommand
        {
            get;
        }

        public string WindowTitle => "ModernWpfUI with Prism";

        #endregion

    }

}