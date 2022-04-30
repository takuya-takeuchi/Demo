using System.Windows.Input;
using Prism.Commands;
using Prism.Navigation;
using Xamarin.OpenCV.Services;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.ViewModels
{

    public class InfoPageViewModel : ViewModelBase, IInfoPageViewModel
    {

        #region Fields

        private readonly IOpenCVService _OpenCvService;

        #endregion

        #region Constructors

        public InfoPageViewModel(INavigationService navigationService, IOpenCVService openCvService)
            : base(navigationService)
        {
            this._OpenCvService = openCvService;

            this.Title = "Info";

            this.Version = openCvService.GetVersion();
        }

        #endregion

        #region Properties

        private ICommand _ShowBuildInformation;

        public ICommand ShowBuildInformation
        {
            get
            {
                return this._ShowBuildInformation ?? (this._ShowBuildInformation = new DelegateCommand(() =>
                {
                    this.BuildInformation = this._OpenCvService.GetBuildInformation();
                }));
            }
        }

        private string _BuildInformation;

        public string BuildInformation
        {
            get => this._BuildInformation;
            private set => this.SetProperty(ref this._BuildInformation, value);
        }

        private string _Title;

        public string Title
        {
            get => this._Title;
            private set => this.SetProperty(ref this._Title, value);
        }

        public string Version
        {
            get;
        }

        #endregion

    }

}
