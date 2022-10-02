using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.ViewModels.Modules
{

    internal sealed class ModuleBViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        #endregion

        #region Constructors

        public ModuleBViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;
            this.Text = "Hello, Module B";
        }

        #endregion

        #region IModuleViewModel Members

        public string Text
        {
            get;
        }

        #endregion

    }

}
