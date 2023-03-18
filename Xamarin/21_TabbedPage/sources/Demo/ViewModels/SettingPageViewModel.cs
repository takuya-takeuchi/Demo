using Prism.Navigation;

using Demo.ViewModels.Interfaces;
using Demo.Services.Interfaces;

namespace Demo.ViewModels
{

    public sealed class SettingPageViewModel : TabbedPageViewModelBase, ISettingPageViewModel
    {

        #region Constructors

        public SettingPageViewModel(INavigationService navigationService,
                                    ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Setting";
        }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnActivated()
        {
            this.LoggingService.Info($"{nameof(SettingPageViewModel)} is activated");
            base.OnActivated();
        }

        protected override void OnDeactivated()
        {
            this.LoggingService.Info($"{nameof(SettingPageViewModel)} is deactivated");
            base.OnDeactivated();
        }

        #endregion

        #endregion

    }

}
