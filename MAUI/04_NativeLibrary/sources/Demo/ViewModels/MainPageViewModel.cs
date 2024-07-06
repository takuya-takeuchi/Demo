using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly INativeService _NativeService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IDialogService dialogService,
                                 INativeService nativeService)
            : base(navigationService, dialogService)
        {
            this._NativeService = nativeService;
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _InvokeNativeCommand;

        public DelegateCommand InvokeNativeCommand
        {
            get
            {
                return this._InvokeNativeCommand ??= new DelegateCommand(async () =>
                {
                    var ret = this._NativeService.Add(1, 2);
                    await this.DialogService.ShowAlert("Info", $"1 + 2 = {ret}", "OK");
                });
            }
        }

        #endregion

    }

}