using Prism.Mvvm;
using Prism.Navigation;

using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    public abstract class ViewModelBase : BindableBase, IInitialize, INavigationAware, IDestructible
    {

        #region Constructors

        protected ViewModelBase(INavigationService navigationService,
                                ILoggingService loggingService)
        {
            this.NavigationService = navigationService;
            this.LoggingService = loggingService;
        }

        #endregion

        #region Properties

        protected ILoggingService LoggingService
        {
            get;
        }

        protected INavigationService NavigationService
        {
            get;
        }

        private string _Title;

        public string Title
        {
            get => this._Title;
            protected set => SetProperty(ref this._Title, value);
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

        #endregion

        #region IDestructible Members

        public virtual void Destroy()
        {

        }

        #endregion

    }

}
