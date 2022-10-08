using System.Collections.ObjectModel;

using Demo.ViewModels;
using Demo.ViewModels.Modules.Interfaces;

namespace Demo.DesignTimes.Modules
{

    internal class MainModuleViewModel : IModuleViewModel
    {

        public ObservableCollection<ListViewItemViewModel> Items
        {
            get;
        }

    }

}
