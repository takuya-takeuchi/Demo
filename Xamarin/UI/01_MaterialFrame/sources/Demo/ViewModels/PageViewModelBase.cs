﻿using Prism.Navigation;

using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    public abstract class PageViewModelBase : ViewModelBase, IInitialize, INavigationAware, IDestructible
    {

        #region Constructors

        protected PageViewModelBase(INavigationService navigationService,
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

        public virtual void Destroy()
        {

        }
        
        #endregion

    }

}
