namespace Source.ViewModels
{

    internal abstract class ViewModelBase : BindableBase
    {

        #region Constructors

        protected ViewModelBase(INavigationService navigationService)
        {
            this.NavigationService = navigationService;
        }

        #endregion

        #region Properties

        protected INavigationService NavigationService
        {
            get;
        }

        #endregion

    }

}