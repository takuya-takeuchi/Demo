using Prism.Navigation;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.ViewModels
{

    public class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {
        
        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            this.Title = "Main Page";
        }

        #endregion

        #region Properties

        private string _Title;

        public string Title
        {
            get => this._Title;
            private set => this.SetProperty(ref this._Title, value);
        }

        #endregion

    }

}
