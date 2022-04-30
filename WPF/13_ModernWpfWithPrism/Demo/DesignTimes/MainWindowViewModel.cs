using System.Collections.Generic;
using System.Windows.Controls;

using ModernWpf.Controls;
using Prism.Commands;

using Demo.Models;
using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    internal sealed class MainWindowViewModel : IMainWindowViewModel
    {

        #region IMainWindowViewModel Membbers

        public IEnumerable<ModuleItem> Modules
        {
            get
            {
                return new[]
                {
                    new ModuleItem(nameof(Demo.Views.Modules.ModuleAView), "Module A"),
                    new ModuleItem(nameof(Demo.Views.Modules.ModuleBView), "Module B")
                };
            }
        }

        public DelegateCommand<ModuleItem> PageListSelectionChanged
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