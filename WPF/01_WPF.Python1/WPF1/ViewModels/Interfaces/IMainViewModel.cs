using GalaSoft.MvvmLight.Command;

namespace WPFPython1.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        RelayCommand MessageCommand
        {
            get;
        }

    }

}
