using System;

using Prism;
using Prism.Mvvm;
using Prism.Navigation;

using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    public abstract class TabbedPageViewModelBase : BindableBase, IActiveAware
    {

        #region Constructors

        protected TabbedPageViewModelBase(INavigationService navigationService,
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

        protected virtual void OnActivated()
        {

        }

        protected virtual void OnDeactivated()
        {

        }

        #endregion

        #region IActiveAware Members

        public event EventHandler IsActiveChanged;

        private bool _IsActive;

        public bool IsActive
        {
            get => this._IsActive;
            set
            {
                if (!this.SetProperty(ref this._IsActive, value)) 
                    return;

                if (this._IsActive)
                    this.OnActivated();
                else
                    this.OnDeactivated();
            }
        }

        #endregion

    }

}
