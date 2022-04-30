using System.Collections.Generic;
using System.Windows.Controls;

using ModernWpf.Controls;
using Prism.Commands;

using Demo.Models;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainWindowViewModel
    {

        #region Properties

        IEnumerable<ModuleItem> Modules
        {
            get;
        }

        DelegateCommand<ModuleItem> PageListSelectionChanged
        {
            get;
        }

        DelegateCommand<AutoSuggestBoxQuerySubmittedEventArgs> QuerySubmittedCommand
        {
            get;
        }

        DelegateCommand<AutoSuggestBoxTextChangedEventArgs> QueryTextChangedCommand
        {
            get;
        }

        DelegateCommand ThemeChangeCommand
        {
            get;
        }

        string WindowTitle
        {
            get;
        }

        #endregion

    }

}