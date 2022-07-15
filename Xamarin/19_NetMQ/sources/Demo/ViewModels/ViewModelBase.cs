using Prism.Mvvm;
using Prism.Navigation;

namespace Demo.ViewModels
{

    public abstract class ViewModelBase : BindableBase, IInitialize, INavigationAware, IDestructible
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

        #region Methods

        public virtual void Initialize(INavigationParameters parameters)
        {

        }

        public virtual void OnNavigatedFrom(INavigationParameters parameters)
        {

        }

        public virtual void OnNavigatedTo(INavigationParameters parameters)
        {

        }

        public virtual void Destroy()
        {

        }
        
        #endregion

    }

}
