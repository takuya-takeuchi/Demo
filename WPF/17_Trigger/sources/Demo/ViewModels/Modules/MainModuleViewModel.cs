using System.Windows;
using System.Windows.Media;

using Prism.Commands;
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

        private bool _Checked;

        public bool Checked
        {
            get => this._Checked;
            set => this.SetProperty(ref this._Checked, value);
        }

        private DelegateCommand _ClickCommand;

        public DelegateCommand ClickCommand
        {
            get
            {
                return this._ClickCommand ??= new DelegateCommand(() =>
                {
                    MessageBox.Show("Hello!!");
                });
            }
        }

        #endregion

    }

}
