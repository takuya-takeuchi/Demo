using System.Collections.ObjectModel;
using System.Threading.Tasks;
using System.Windows.Input;
using System;
using System.Collections;

using Xamarin.Forms;

using Prism.Navigation;

using Demo.Models;
using Demo.Services;
using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MediaPageViewModel : TabbedPageViewModelBase, IMediaPageViewModel
    {

        #region Fields

        private readonly IMediaService _MediaService;

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public MediaPageViewModel(INavigationService navigationService,
                                  IMediaService mediaService,
                                  ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Media";

            this._MediaService = mediaService;
            this._LoggingService = loggingService;

            this.ItemTappedCommand = new Command<MediaAsset>(this.OnItemTapped);
            this.MediaAssets = new ObservableCollection<MediaAsset>();
            BindingBase.EnableCollectionSynchronization(this.MediaAssets, null, ObservableCollectionCallback);
            this._MediaService.OnMediaAssetLoaded += this.OnMediaAssetLoaded;
        }

        #endregion

        #region Methods

        #region Overrides

        protected override async void OnActivated()
        {
            this.LoggingService.Info($"{nameof(MediaPageViewModel)} is activated");
            
            this.MediaAssets.Clear();
            await this.LoadMediaAssets();

            base.OnActivated();
        }

        protected override void OnDeactivated()
        {
            this.LoggingService.Info($"{nameof(MediaPageViewModel)} is deactivated");
            base.OnDeactivated();
        }

        #endregion

        #region Event Handlers

        private void OnMediaAssetLoaded(object sender, MediaEventArgs e)
        {
            this.MediaAssets.Add(e.Media);
        }

        #endregion

        #region Helpers

        private async Task LoadMediaAssets()
        {
            try
            {
                await this._MediaService.RetrieveMediaAssetsAsync();
            }
            catch (TaskCanceledException)
            {
                this._LoggingService.Info("Task was cancelled");
            }
        }

        private static void ObservableCollectionCallback(IEnumerable collection, object context, Action accessMethod, bool writeAccess)
        {
            // `lock` ensures that only one thread access the collection at a time
            lock (collection)
                accessMethod?.Invoke();
        }

        private async void OnItemTapped(MediaAsset mediaAsset)
        {
            this._LoggingService.Info($"Thumbnail is tapped. Id: {mediaAsset.Id}, Name: {mediaAsset.Name}, Type: {mediaAsset.Type}");
            //if (mediaAsset.Type == MediaAssetType.Video)
            //{
            //    await App.Current.MainPage.Navigation.PushAsync(new VideoDetailPage(mediaAsset.Path));
            //}
            //else
            //{
            //    await App.Current.MainPage.Navigation.PushAsync(new ImageDetailPage(mediaAsset.Path));
            //}
        }

        #endregion

        #endregion

        #region IMediaPageViewModel Members

        public ICommand ItemTappedCommand { get; }

        public ObservableCollection<MediaAsset> MediaAssets { get; set; }

        private MediaAsset _MediaSelected;

        public MediaAsset MediaSelected
        {
            get => this._MediaSelected;
            set
            {
                this._MediaSelected = value;
                if (this._MediaSelected != null)
                    OnItemTapped(this._MediaSelected);
            }
        }

        public string SearchText { get; set; }

        #endregion

    }

}
