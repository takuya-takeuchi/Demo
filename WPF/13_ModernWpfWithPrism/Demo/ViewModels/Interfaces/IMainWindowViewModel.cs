using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Input;

using ModernWpf.Controls;
using Prism.Commands;
using Prism.Mvvm;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainWindowViewModel
    {

        #region Properties

        DelegateCommand<SelectionChangedEventArgs> PageListSelectionChanged
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