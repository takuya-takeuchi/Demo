using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.ViewModels.Modules
{

    internal sealed class ModuleAViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        #endregion

        #region Constructors

        public ModuleAViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;
            this.Text = "Hello, Module A";
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
