using System.Windows.Input;
using Prism.Commands;
using Prism.Navigation;
using Xamarin.Camera.ViewModels.Interfaces;

namespace Xamarin.Camera.ViewModels
{

    public sealed class ScreenPageViewModel : ViewModelBase, IScreenPageViewModel
    {

        #region Constructors

        public ScreenPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            this.Title = "Screen";
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

        #region Methods

        #region Overrids

        #endregion

        #region Event Handlers

        #endregion

        #region Helpers

        #endregion

        #endregion

    }

}
