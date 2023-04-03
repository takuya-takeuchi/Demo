using System;
using System.IO;

using Xamarin.Forms;

using Plugin.Media;
using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Main Page";
        }

        #endregion

        #region Properties

        private ImageSource _PhotoImage;

        public ImageSource PhotoImage
        {
            get => this._PhotoImage;
            private set => this.SetProperty(ref this._PhotoImage, value);
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

        private DelegateCommand _TakePhotoCommand;

        public DelegateCommand TakePhotoCommand => this._TakePhotoCommand ?? (this._TakePhotoCommand = new DelegateCommand(this.ExecuteTakePhoto));

        #endregion

        #region Methods

        #region Helpers
        
        private async void ExecuteTakePhoto()
        {
            if (!CrossMedia.Current.IsCameraAvailable || !CrossMedia.Current.IsTakePhotoSupported)
            {
                this.LoggingService.Error("No Camera");
                return;
            }

            var guid = Guid.NewGuid();
            var file = await CrossMedia.Current.TakePhotoAsync(new Plugin.Media.Abstractions.StoreCameraMediaOptions
            {
                Directory = "Sample",
                Name = $"{guid}.jpg",
                SaveToAlbum = false
            });

            if (file == null)
                return;

            this.PhotoImage = ImageSource.FromStream(() =>
            {
                var stream = file.GetStream();
                return stream;
            });

            try
            {
                File.Delete(file.Path);
                var exist = File.Exists(file.Path);
                this.LoggingService.Info(exist ? $"{file.Path} was not deleted" : $"{file.Path} is deleted");
            }
            catch (Exception ex)
            {
                this.LoggingService.Error(ex, null, "Failed to delete temp photo image");
            }
        }

        #endregion

        #endregion

    }

}