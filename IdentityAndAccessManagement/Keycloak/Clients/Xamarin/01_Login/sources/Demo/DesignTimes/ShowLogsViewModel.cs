using Demo.ViewModels.Interfaces;
using Prism.Commands;

namespace Demo.DesignTimes
{

    public sealed class ShowLogsViewModel : IShowLogsViewModel
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