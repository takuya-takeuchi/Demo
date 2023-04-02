using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        DelegateCommand ReloadCommand
        {
            get;
        }

        DelegateCommand ResetCommand
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        string Key
        {
            get;
        }

    }

}