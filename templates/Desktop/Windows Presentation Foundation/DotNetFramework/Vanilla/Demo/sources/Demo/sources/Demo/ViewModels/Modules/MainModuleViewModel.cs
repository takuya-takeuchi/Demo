using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Modules.Interfaces;
using NLog;

namespace Demo.ViewModels.Modules
{

    internal sealed class MainModuleViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Constructors

        public MainModuleViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;
            Logger.Info($"Constructor of {nameof(MainModuleViewModel)}");
        }

        #endregion

        #region IModuleViewModel Members
        #endregion

    }

}
