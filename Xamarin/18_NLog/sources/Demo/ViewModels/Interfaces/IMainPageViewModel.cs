using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel
    {

        DelegateCommand<string> LoggingCommand
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        string Title
        {
            get;
        }

    }

}