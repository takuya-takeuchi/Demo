using GalaSoft.MvvmLight.Command;

namespace WPFPython2.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        RelayCommand MessageCommand
        {
            get;
        }

        int Start
        {
            get;
            set;
        }

        int End
        {
            get;
            set;
        }

    }

}
