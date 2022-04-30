using Prism.Commands;

namespace UWP.ShortCutKey.ViewModels.Interfaces
{

    public interface IMainPageViewModel
    {

        DelegateCommand SaveCommand
        {
            get;
        }

        DelegateCommand NewCommand
        {
            get;
        }

    }

}