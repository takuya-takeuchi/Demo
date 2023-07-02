using System;
using Microsoft.Maui.ApplicationModel;

using Source.ViewModels.Interfaces;

namespace Source.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
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
                        var uri = new Uri("https://taktak.jp/buy");
                        await Browser.Default.OpenAsync(uri, BrowserLaunchMode.SystemPreferred);
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