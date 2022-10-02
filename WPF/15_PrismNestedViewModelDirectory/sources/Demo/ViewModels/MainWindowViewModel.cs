using Prism.Commands;
using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Interfaces;

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
            this.ShowViewCommand = new DelegateCommand<string>(this.ShowView);
        }

        #endregion

        #region Methods

        #region Event Handlers

        public void ShowView(string viewName)
        {
            this._RegionManager.RequestNavigate("ContentRegion", viewName);
        }

        #endregion

        #endregion

        #region IMainWindowViewModel Members

        public DelegateCommand<string> ShowViewCommand
        {
            get;
        }

        #endregion

    }

}
