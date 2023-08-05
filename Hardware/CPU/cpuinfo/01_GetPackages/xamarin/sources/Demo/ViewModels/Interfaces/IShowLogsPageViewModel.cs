using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IShowLogsPageViewModel : IViewModel
    {

        DelegateCommand ClearCommand
        {
            get;
        }

        string Logs
        {
            get;
        }

    }

}