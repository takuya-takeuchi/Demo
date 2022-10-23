using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IFolderPickerService _FolderPickerService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IFolderPickerService folderPickerService)
            : base(navigationService)
        {
            this._FolderPickerService = folderPickerService;
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _OpenFolderCommand;

        public DelegateCommand OpenFolderCommand
        {
            get
            {
                return this._OpenFolderCommand ??= new DelegateCommand(async () =>
                {
                    var ret = await this._FolderPickerService.PickFolder();
                });
            }
        }

        #endregion

    }

}