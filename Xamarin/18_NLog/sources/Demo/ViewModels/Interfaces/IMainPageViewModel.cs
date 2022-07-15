using System.Collections.ObjectModel;
using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel
    {

        DelegateCommand<string> LoggingCommand
        {
            get;
        }

        DelegateCommand LoadCommand
        {
            get;
        }

        string Log
        {
            get;
        }

        string Title
        {
            get;
        }

    }

}