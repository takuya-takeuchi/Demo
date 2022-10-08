using Prism.Mvvm;
using Prism.Regions;

using Demo.ViewModels.Modules.Interfaces;
using System.Collections.ObjectModel;

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

            this.Items = new ObservableCollection<ListViewItemViewModel>();
            this.Items.Add(new ListViewItemViewModel("1")
            {
                Children =
                {
                    new ListViewItemViewModel("1-1")
                    {
                        Children = { new ListViewItemViewModel("1-1-1") }
                    },
                    new ListViewItemViewModel("1-2")
                    {
                        Children = { new ListViewItemViewModel("1-2-1") }
                    }
                }
            });
            this.Items.Add(new ListViewItemViewModel("2")
            {
                Children =
                {
                    new ListViewItemViewModel("2-1"),
                    new ListViewItemViewModel("2-2"),
                }
            });
        }

        #endregion

        #region IModuleViewModel Members

        public ObservableCollection<ListViewItemViewModel> Items
        {
            get;
        }

        #endregion

    }

}
