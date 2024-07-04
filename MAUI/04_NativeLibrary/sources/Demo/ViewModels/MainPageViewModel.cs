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
                                 INativeService nativeService)
            : base(navigationService)
        {
            this._NativeService = nativeService;
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _OpenFolderCommand;

        public DelegateCommand OpenFolderCommand
        {
            get
            {
                return this._OpenFolderCommand ??= new DelegateCommand(() =>
                {
                    var ret = this._NativeService.Add(1, 2);
                });
            }
        }

        #endregion

    }

}