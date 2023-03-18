using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using System.Threading;

using Android;
using Android.App;
using Android.Content.PM;
using Android.Database;
using Android.Graphics;
using Android.OS;
using Android.Provider;
using Android.Runtime;
using AndroidX.Core.App;
using AndroidX.Core.Content;

using Acr.UserDialogs;
using Xamarin.Essentials;

using Demo.Models;
using Demo.Services;
using Demo.Droid.Helpers;
using Demo.Services.Interfaces;

namespace Demo.Droid.Services
{

    internal sealed class MediaService : IMediaService
    {
        
        #region Fields

        private bool _StopLoad;

        private static TaskCompletionSource<bool> MediaPermissionTcs;

        public const int RequestMedia = 1354;

        #endregion

        #region Methods

        #region Helpers

        private static string GetString(ICursor cursor, string key)
        {
            return cursor.GetString(cursor.GetColumnIndex(key));
        }

        private async Task<IList<MediaAsset>> LoadMediaAsync()
        {
            IList<MediaAsset> assets = new List<MediaAsset>();
            var hasPermission = await RequestPermissionAsync();
            if (!hasPermission)
                return assets;

            var uri = MediaStore.Files.GetContentUri("external");
            var ctx = Application.Context;
            await Task.Run(async () =>
            {
                var cursor = ctx.ApplicationContext.ContentResolver.Query(uri, new string[]
                {
                    MediaStore.Files.FileColumns.Id,
                    MediaStore.Files.FileColumns.Data,
                    MediaStore.Files.FileColumns.DateAdded,
                    MediaStore.Files.FileColumns.MediaType,
                    MediaStore.Files.FileColumns.MimeType,
                    MediaStore.Files.FileColumns.Title,
                    MediaStore.Files.FileColumns.Parent,
                    MediaStore.Files.FileColumns.DisplayName,
                    MediaStore.Files.FileColumns.Size
                }, $"{MediaStore.Files.FileColumns.MediaType} = {(int)MediaType.Image} OR {MediaStore.Files.FileColumns.MediaType} = {(int)MediaType.Video}", null, $"{MediaStore.Files.FileColumns.DateAdded} DESC");
                if (cursor.Count > 0)
                {
                    while (cursor.MoveToNext())
                    {
                        try
                        {
                            var id = cursor.GetInt(cursor.GetColumnIndex(MediaStore.Files.FileColumns.Id));
                            var mediaType = cursor.GetInt(cursor.GetColumnIndex(MediaStore.Files.FileColumns.MediaType));

                            Bitmap bitmap = null;
                            switch (mediaType)
                            {
                                case (int)MediaType.Image:
                                    bitmap = MediaStore.Images.Thumbnails.GetThumbnail(Platform.CurrentActivity.ContentResolver, id, ThumbnailKind.MiniKind, new BitmapFactory.Options()
                                    {
                                        InSampleSize = 4,
                                        InPurgeable = true
                                    });
                                    break;
                                case (int)MediaType.Video:
                                    bitmap = MediaStore.Video.Thumbnails.GetThumbnail(Platform.CurrentActivity.ContentResolver, id, VideoThumbnailKind.MiniKind, new BitmapFactory.Options()
                                    {
                                        InSampleSize = 4,
                                        InPurgeable = true
                                    });
                                    break;
                            }

                            var tmpPath = System.IO.Path.GetTempPath();
                            var name = GetString(cursor, MediaStore.Files.FileColumns.DisplayName);
                            var filePath = System.IO.Path.Combine(tmpPath, $"tmp_{name}");

                            var path = GetString(cursor, MediaStore.Files.FileColumns.Data);

                            await using (var stream = new FileStream(filePath, FileMode.Create))
                            {
                                await bitmap?.CompressAsync(Bitmap.CompressFormat.Png, 100, stream);
                                stream.Close();
                            }

                            if (!string.IsNullOrWhiteSpace(filePath))
                            {
                                var asset = new MediaAsset(id: $"{id}",
                                                           type: mediaType == (int)MediaType.Video ? MediaAssetType.Video : MediaAssetType.Image,
                                                           name: name,
                                                           previewPath: filePath,
                                                           path: path);

                                using (var h = new Handler(Looper.MainLooper))
                                    h.Post(async () => { this.OnMediaAssetLoaded?.Invoke(this, new MediaEventArgs(asset)); });

                                assets.Add(asset);
                            }

                            if (this._StopLoad)
                                break;
                        }
                        catch (Exception ex)
                        {
                            await UserDialogs.Instance.AlertAsync(ex.StackTrace, "error", "ok");
                        }
                    }
                }
            });

            return assets;
        }

        private static void OnRequestPermissionsResult(int requestCode, string[] permissions, [GeneratedEnum] Android.Content.PM.Permission[] grantResults)
        {
            if (requestCode != MediaService.RequestMedia)
                return;

            // We have requested multiple permissions for Media, so all of them need to be
            // checked.
            if (PermissionHelper.VerifyPermissions(grantResults))
            {
                // All required permissions have been granted, display Media fragment.
                MediaPermissionTcs.TrySetResult(true);
            }
            else
            {
                MediaPermissionTcs.TrySetResult(false);
            }
        }

        private static async void RequestMediaPermissions()
        {
            if (ActivityCompat.ShouldShowRequestPermissionRationale(Platform.CurrentActivity, Manifest.Permission.WriteExternalStorage))
            {

                // Provide an additional rationale to the user if the permission was not granted
                // and the user would benefit from additional context for the use of the permission.
                // For example, if the request has been denied previously.

                await UserDialogs.Instance.AlertAsync("Media Permission", "This action requires external storafge permission", "Ok");
            }
            else
            {
                // Media permissions have not been granted yet. Request them directly.
                ActivityCompat.RequestPermissions(Platform.CurrentActivity, new string[] { Manifest.Permission.WriteExternalStorage }, RequestMedia);
            }
        }

        private static async Task<bool> RequestPermissionAsync()
        {
            MediaPermissionTcs = new TaskCompletionSource<bool>();

            // Verify that all required Media permissions have been granted.
            if (ContextCompat.CheckSelfPermission(Platform.CurrentActivity, Manifest.Permission.WriteExternalStorage) != (int)Permission.Granted)
            {
                // Media permissions have not been granted.
                RequestMediaPermissions();
            }
            else
            {
                // Media permissions have been granted. 
                MediaPermissionTcs.TrySetResult(true);
            }

            return await MediaPermissionTcs.Task;
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
            this._StopLoad = false;

            if (!cancelToken.HasValue)
                cancelToken = CancellationToken.None;

            // We create a TaskCompletionSource of decimal
            var taskCompletionSource = new TaskCompletionSource<IList<MediaAsset>>();

            // Registering a lambda into the cancellationToken
            cancelToken.Value.Register(() =>
            {
                // We received a cancellation message, cancel the TaskCompletionSource.Task
                this._StopLoad = true;
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