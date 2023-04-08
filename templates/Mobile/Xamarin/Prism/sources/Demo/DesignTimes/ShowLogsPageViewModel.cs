using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class ShowLogsPageViewModel : IShowLogsPageViewModel
    {

        public DelegateCommand ClearCommand
        {
            get;
        }

        public string Logs
        {
            get;
            set;
        }

        public string Title
        {
            get;
        }

    }

}