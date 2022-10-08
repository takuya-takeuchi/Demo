using Prism.Commands;
using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Interfaces;
using System.Windows.Controls;
using Demo.Views.Modules;

namespace Demo.ViewModels
{

    internal sealed class MainWindowViewModel : BindableBase, IMainWindowViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        #endregion

        #region Constructors

        public MainWindowViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;

            this.LoadedCommand = new DelegateCommand(() =>
            {
                this._RegionManager.RequestNavigate(this.RegionName, nameof(MainModule));
            });
        }

        #endregion

        #region IMainWindowViewModel Members

        public DelegateCommand LoadedCommand
        {
            get;
        }

        public string RegionName => nameof(MainWindowViewModel);

        #endregion

    }

}
