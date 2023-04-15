using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;
using Demo.Controls;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IDeviceOrientationDetectService _DeviceOrientationDetectService;

        private readonly IPermissionService _PermissionService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IDeviceOrientationDetectService deviceOrientationDetectService,
                                 IPermissionService permissionService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this._DeviceOrientationDetectService = deviceOrientationDetectService;
            this._PermissionService = permissionService;

            deviceOrientationDetectService.OrientationChanged +=this. OnOrientationChanged;

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

        private Orientation _CameraOrientation;

        public Orientation CameraOrientation
        {
            get => this._CameraOrientation;
            private set => this.SetProperty(ref this._CameraOrientation, value);
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

        #region EventHandlers

        private void OnOrientationChanged(object sender, Orientation e)
        {
            this.CameraOrientation = e;
        }

        #endregion

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