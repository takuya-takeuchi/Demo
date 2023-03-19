using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface ILoginPageViewModel : IViewModel
    {

        bool IsLoggedIn
        {
            get;
        }

        DelegateCommand LoginCommand
        {
            get;
        }

    }

}