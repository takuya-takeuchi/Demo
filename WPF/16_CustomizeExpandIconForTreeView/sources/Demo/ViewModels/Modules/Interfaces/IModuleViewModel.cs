using System.Collections.ObjectModel;

namespace Demo.ViewModels.Modules.Interfaces
{

    internal interface IModuleViewModel
    {

        ObservableCollection<ListViewItemViewModel> Items
        {
            get;
        }

    }

}