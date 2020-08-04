using Prism.Navigation;
using Xamarin.Native.Nuget.ViewModels.Interfaces;

namespace Xamarin.Native.Nuget.ViewModels
{

    public class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly INativeService _NativeService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 INativeService nativeService)
            : base(navigationService)
        {
            this._NativeService = nativeService;
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