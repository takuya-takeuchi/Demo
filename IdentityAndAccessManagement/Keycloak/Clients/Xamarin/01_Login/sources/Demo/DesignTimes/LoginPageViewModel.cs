using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class LoginPageViewModel : ILoginPageViewModel
    {

        public bool IsLoggedIn
        {
            get;
        }

        public DelegateCommand LoginCommand
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}