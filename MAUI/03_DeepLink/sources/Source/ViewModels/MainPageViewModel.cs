using System;

using Source.Services.Interfaces;
using Source.ViewModels.Interfaces;

namespace Source.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IDeepLinkService _DeepLinkService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService, IDeepLinkService deepLinkService)
            : base(navigationService)
        {
            this._DeepLinkService = deepLinkService;
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _OpenApplicationCommand;

        public DelegateCommand OpenApplicationCommand
        {
            get
            {
                return this._OpenApplicationCommand ??= new DelegateCommand(async () =>
                {
                    try
                    {
                        await this._DeepLinkService.Open();
                    }
                    catch (Exception ex)
                    {
                        // An unexpected error occurred. No browser may be installed on the device.
                    }
                });
            }
        }

        #endregion

    }

}