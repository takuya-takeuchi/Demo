using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        DelegateCommand ShowLogCommand
        {
            get;
        }

        string Message
        {
            get;
        }

    }

}