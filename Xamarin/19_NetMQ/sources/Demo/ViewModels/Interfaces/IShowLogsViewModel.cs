using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IShowLogsViewModel : IViewModel
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