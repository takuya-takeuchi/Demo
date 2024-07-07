using Prism.Mvvm;
using Prism.Navigation;

using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    internal abstract class ViewModelBase : BindableBase
    {

        #region Constructors

        protected ViewModelBase(INavigationService navigationService,
                                IDialogService dialogService)
        {
            this.NavigationService = navigationService;
            this.DialogService = dialogService;
        }

        #endregion

        #region Properties

        protected IDialogService DialogService
        {
            get;
        }

        protected INavigationService NavigationService
        {
            get;
        }

        #endregion

    }

}