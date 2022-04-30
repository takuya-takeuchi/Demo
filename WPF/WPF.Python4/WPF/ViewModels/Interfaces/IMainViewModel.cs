using GalaSoft.MvvmLight.Command;

namespace WPFPython.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        RelayCommand MessageCommand
        {
            get;
        }

    }

}
