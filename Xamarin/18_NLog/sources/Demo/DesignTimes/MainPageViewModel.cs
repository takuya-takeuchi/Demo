using System.Collections.ObjectModel;
using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public DelegateCommand<string> LoggingCommand
        {
            get;
        }

        public DelegateCommand LoadCommand
        {
            get;
        }

        public string Log
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
