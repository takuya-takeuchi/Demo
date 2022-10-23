using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields
        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
        }

        #endregion

        #region Properties
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