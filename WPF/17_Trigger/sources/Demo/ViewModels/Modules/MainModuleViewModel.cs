using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.ViewModels.Modules
{

    internal sealed class MainModuleViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        #endregion

        #region Constructors

        public MainModuleViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;
        }

        #endregion

        #region IModuleViewModel Members
        #endregion

    }

}
