using System;
using System.Collections.Generic;
using System.Threading.Tasks;

using Foundation;
using Microsoft.Maui.ApplicationModel;
using UIKit;

namespace Demo.Platforms.iOS.Services
{

    internal sealed class FolderPickerService : IFolderPickerService
    {

        #region Methods

        #region Helpers

        private static void GetFileResults(IReadOnlyList<NSUrl> urls, TaskCompletionSource<string> tcs)
        {
            try
            {
                tcs.TrySetResult(urls?[0]?.ToString() ?? "");
            }
            catch (Exception ex)
            {
                tcs.TrySetException(ex);
            }
        }

        #endregion

        #endregion

        #region IFolderPickerService Members

        public async Task<string> PickFolder()
        {
            var allowedTypes = new[]
            {
                "public.folder"
            };

            var tcs = new TaskCompletionSource<string>();
            var picker = new UIDocumentPickerViewController(allowedTypes, UIDocumentPickerMode.Open);
            picker.Delegate = new PickerDelegate
            {
                PickHandler = urls => GetFileResults(urls, tcs)
            };
            picker.PresentationController.Delegate = new UIPresentationControllerDelegate(() =>
            {
                GetFileResults(null, tcs);
            });

            var parentController = Platform.GetCurrentUIViewController();
            parentController?.PresentViewController(picker, true, null);

            return await tcs.Task;
        }

        #endregion

        private sealed class PickerDelegate : UIDocumentPickerDelegate
        {

            #region Constructors

            #endregion

            #region Properties

            public Action<NSUrl[]> PickHandler
            {
                get;
                set;
            }

            #endregion

            #region Methods

            #region Overrids

            public override void WasCancelled(UIDocumentPickerViewController controller)
            {
                this.PickHandler?.Invoke(null);
            }

            public override void DidPickDocument(UIDocumentPickerViewController controller, NSUrl[] urls)
            {
                this.PickHandler?.Invoke(urls);
            }

            public override void DidPickDocument(UIDocumentPickerViewController controller, NSUrl url)
            {
                this.PickHandler?.Invoke(new[] { url });
            }

            #endregion

            #endregion

        }

        private sealed class UIPresentationControllerDelegate : UIAdaptivePresentationControllerDelegate
        {

            #region Fields

            private Action _DismissHandler;

            #endregion

            #region Constructors

            internal UIPresentationControllerDelegate(Action dismissHandler)
            {
                this._DismissHandler = dismissHandler;
            }

            #endregion

            #region Methods

            #region Overrids

            public override void DidDismiss(UIPresentationController presentationController)
            {
                this._DismissHandler?.Invoke();
                this._DismissHandler = null;
            }

            protected override void Dispose(bool disposing)
            {
                this._DismissHandler?.Invoke();
                base.Dispose(disposing);
            }

            #endregion

            #endregion

        }

    }

}
