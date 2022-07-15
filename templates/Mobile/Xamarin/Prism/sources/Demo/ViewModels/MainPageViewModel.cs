using Demo.ViewModels.Interfaces;
using Prism.Navigation;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
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
