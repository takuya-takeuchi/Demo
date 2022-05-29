using System.IO;
using System.Windows.Input;
using Prism.Commands;
using Prism.Navigation;
using Xamarin.Forms;
using Xamarin.OpenCV.Services;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.ViewModels
{

    public class PhotoPageViewModel : ViewModelBase, IPhotoPageViewModel
    {

        #region Fields

        private readonly IPhotoPickerService _PhotoPickerService;

        private readonly IOpenCVService _OpenCvService;

        #endregion

        #region Constructors

        public PhotoPageViewModel(INavigationService navigationService,
                                  IPhotoPickerService photoPickerService,
                                  IOpenCVService openCvService)
            : base(navigationService)
        {
            this._PhotoPickerService = photoPickerService;
            this._OpenCvService = openCvService;

            this.Title = "Photo";
        }

        #endregion

        #region Properties

        private ICommand _ShowPhoto;

        public ICommand ShowPhoto
        {
            get
            {
                return this._ShowPhoto ?? (this._ShowPhoto = new DelegateCommand(async () =>
                {
                    // Do not dispose
                    var stream = await this._PhotoPickerService.GetImageStreamAsync();
                    if (stream != null)
                    {
                        this.Photo = ImageSource.FromStream(() => stream);
                    }
                }));
            }
        }

        private string _Title;

        public string Title
        {
            get => this._Title;
            private set => this.SetProperty(ref this._Title, value);
        }

        private ImageSource _Photo;

        public ImageSource Photo
        {
            get => this._Photo;
            private set => this.SetProperty(ref this._Photo, value);
        }

        #endregion

    }

}
