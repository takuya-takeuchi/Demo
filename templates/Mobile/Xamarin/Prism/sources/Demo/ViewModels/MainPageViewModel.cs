using Demo.ViewModels.Interfaces;
using Prism.Navigation;

using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService)
        {
            this._LoggingService = loggingService;
            Title = "Main Page";
        }

        #endregion

        #region Properties

        private string _Title;

        public string Title
        {
            get => this._Title;
            set => SetProperty(ref this._Title, value);
        }

        #endregion

    }

}
