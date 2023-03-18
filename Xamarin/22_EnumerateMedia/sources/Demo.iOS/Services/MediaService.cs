using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Threading;

using AVFoundation;
using CoreGraphics;
using Foundation;
using Photos;
using UIKit;

using Demo.Models;
using Demo.Services;
using Demo.Services.Interfaces;

namespace Demo.iOS.Services
{

    internal sealed class MediaService : IMediaService
    {

        #region Fields

        private bool _RequestStop;

        #endregion

        #region Methods

        #region Helpers

        private async Task<IList<MediaAsset>> LoadMediaAsync()
        {
            IList<MediaAsset> assets = new List<MediaAsset>();
            var imageManager = new PHCachingImageManager();
            var hasPermission = await RequestPermissionAsync();
            if (!hasPermission)
                return assets;

            await Task.Run(async () =>
            {
                var thumbnailRequestOptions = new PHImageRequestOptions();
                thumbnailRequestOptions.ResizeMode = PHImageRequestOptionsResizeMode.Fast;
                thumbnailRequestOptions.DeliveryMode = PHImageRequestOptionsDeliveryMode.FastFormat;
                thumbnailRequestOptions.NetworkAccessAllowed = true;
                thumbnailRequestOptions.Synchronous = true;

                var requestOptions = new PHImageRequestOptions();
                requestOptions.ResizeMode = PHImageRequestOptionsResizeMode.Exact;
                requestOptions.DeliveryMode = PHImageRequestOptionsDeliveryMode.HighQualityFormat;
                requestOptions.NetworkAccessAllowed = true;
                requestOptions.Synchronous = true;

                var fetchOptions = new PHFetchOptions();
                fetchOptions.SortDescriptors = new NSSortDescriptor[] { new NSSortDescriptor("creationDate", false) };
                fetchOptions.Predicate = NSPredicate.FromFormat($"mediaType == {(int)PHAssetMediaType.Image} || mediaType == {(int)PHAssetMediaType.Video}");
                var fetchResults = PHAsset.FetchAssets(fetchOptions);
                var tmpPath = Path.GetTempPath();
                var allAssets = fetchResults.Select(p => p as PHAsset).ToArray();
                var thumbnailSize = new CGSize(300.0f, 300.0f);

                imageManager.StartCaching(allAssets, thumbnailSize, PHImageContentMode.AspectFit, thumbnailRequestOptions);
                imageManager.StartCaching(allAssets, PHImageManager.MaximumSize, PHImageContentMode.AspectFit, requestOptions);
                
                foreach (var result in fetchResults)
                {
                    var phAsset = (result as PHAsset);
                    var name = PHAssetResource.GetAssetResources(phAsset)?.FirstOrDefault()?.OriginalFilename;
                    var id = phAsset.LocalIdentifier;
                    var type = phAsset.MediaType == PHAssetMediaType.Image ? MediaAssetType.Image : MediaAssetType.Video;
                    var path = "";
                    var previewPath = "";

                    imageManager.RequestImageForAsset(phAsset, thumbnailSize, PHImageContentMode.AspectFit, thumbnailRequestOptions, (image, info) =>
                    {
                        var imageData = image.CGImage.RenderingIntent == CGColorRenderingIntent.Default ? image.AsJPEG(0.8f) : image.AsPNG();
                        if (imageData == null)
                            return;

                        var fileName = Path.Combine(tmpPath, $"tmp_thumbnail_{Path.GetFileNameWithoutExtension(name)}.jpg");
                        imageData.Save(fileName, true, out var error);
                        if (error == null)
                            previewPath = fileName;
                    });

                    switch (phAsset.MediaType)
                    {

                        case PHAssetMediaType.Image:

                            imageManager.RequestImageForAsset(phAsset, PHImageManager.MaximumSize, PHImageContentMode.AspectFit, requestOptions, (image, info) =>
                            {
                                var imageData = image.CGImage.RenderingIntent == CGColorRenderingIntent.Default ? image.AsJPEG(0.8f) : image.AsPNG();
                                if (imageData == null) 
                                    return;

                                var fileName = Path.Combine(tmpPath, $"tmp_{name}");
                                imageData.Save(fileName, true, out var error);
                                if (error == null)
                                    path = fileName;
                            });
                            break;
                        case PHAssetMediaType.Video:
                            var videoRequestOptions = new PHVideoRequestOptions();
                            videoRequestOptions.NetworkAccessAllowed = true;
                            var tcs = new TaskCompletionSource<bool>();
                            imageManager.RequestAvAsset(phAsset, null, (vAsset, audioMix, info) =>
                            {
                                if (!(vAsset is AVUrlAsset avAsset))
                                    return;
                                
                                var avData = NSData.FromUrl(avAsset.Url);
                                var tmp = Path.Combine(tmpPath, $"tmp_{name}");

                                avData.Save(path, true, out var error);
                                if (error == null)
                                {
                                    path = tmp;
                                    tcs.TrySetResult(true);
                                }
                                else
                                {
                                    tcs.TrySetResult(false);
                                }
                            });

                            await tcs.Task;
                            break;
                    }

                    var asset = new MediaAsset(id, type, name, previewPath, path);
                    UIApplication.SharedApplication.InvokeOnMainThread(delegate
                    {
                        this.OnMediaAssetLoaded?.Invoke(this, new MediaEventArgs(asset));
                    });
                    assets.Add(asset);

                    if (this._RequestStop)
                        break;
                }
            });

            imageManager.StopCaching();

            return assets;
        }
        
        private static async Task<bool> RequestPermissionAsync()
        {
            var status = PHPhotoLibrary.AuthorizationStatus;

            var authorization = status == PHAuthorizationStatus.Authorized;
            if (!authorization)
                authorization = await PHPhotoLibrary.RequestAuthorizationAsync() == PHAuthorizationStatus.Authorized;

            return authorization;

        }

        #endregion

        #endregion

        #region IMediaService Members

        public event EventHandler<MediaEventArgs> OnMediaAssetLoaded;

        public bool IsLoading
        {
            get;
            private set;
        }

        public async Task<IList<MediaAsset>> RetrieveMediaAssetsAsync(CancellationToken? cancelToken = null)
        {
            this._RequestStop = false;

            if (!cancelToken.HasValue)
                cancelToken = CancellationToken.None;

            // We create a TaskCompletionSource of decimal
            var taskCompletionSource = new TaskCompletionSource<IList<MediaAsset>>();

            // Registering a lambda into the cancellationToken
            cancelToken.Value.Register(() =>
            {
                this._RequestStop = true;
                taskCompletionSource.TrySetCanceled();
            });

            this.IsLoading = true;

            var task = LoadMediaAsync();

            // Wait for the first task to finish among the two
            var completedTask = await Task.WhenAny(task, taskCompletionSource.Task);
            this.IsLoading = false;

            return await completedTask;
        }

        #endregion

    }

}