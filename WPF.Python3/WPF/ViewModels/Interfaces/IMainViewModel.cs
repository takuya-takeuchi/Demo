using System.Collections.ObjectModel;
using GalaSoft.MvvmLight.Command;

namespace WPFPython.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        RelayCommand MessageCommand
        {
            get;
        }

        string SelectedKey
        {
            get;
            set;
        }

        ObservableCollection<string> Keys
        {
            get;
        }

    }

}
