using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields
        
        private readonly IPermissionService _PermissionService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IPermissionService permissionService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this._PermissionService = permissionService;

            this.Title = "Main Page";
        }

        #endregion

        #region Properties

        private bool _CameraOpened;

        public bool CameraOpened
        {
            get => this._CameraOpened;
            private set => this.SetProperty(ref this._CameraOpened, value);
        }

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.NavigationService.NavigateAsync("ShowLogs");
                }));
            }
        }

        private DelegateCommand _CameraPreviewCommand;

        public DelegateCommand CameraPreviewCommand => this._CameraPreviewCommand ?? (this._CameraPreviewCommand = new DelegateCommand(this.ExecuteCameraPreview));

        #endregion

        #region Methods

        #region Helpers
        
        private async void ExecuteCameraPreview()
        {
            if (!this._PermissionService.CheckCameraPermission())
            {
                await this._PermissionService.RequestCameraPermission();
            }
            
            this.CameraOpened = !this._CameraOpened;
        }

        #endregion

        #endregion

    }

}