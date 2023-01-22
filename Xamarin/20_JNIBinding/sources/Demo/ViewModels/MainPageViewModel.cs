using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 INativeService nativeService)
            : base(navigationService)
        {
            this.Title = "Main Page";
            this.Message = nativeService.Message();
        }

        #endregion

        #region IMainPageViewModel Members

        private string _Message;

        public string Message
        {
            get => this._Message;
            private set => SetProperty(ref this._Message, value);
        }

        #endregion

    }

}
