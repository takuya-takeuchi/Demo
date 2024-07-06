using System;
using System.Threading.Tasks;

using Microsoft.Maui;
using Windows.Storage.Pickers;

using Demo.Services.Interfaces;

namespace Demo.Platforms.Windows.Services
{

    internal sealed class FolderPickerService : IFolderPickerService
    {

        #region IFolderPickerService Members

        public async Task<string> PickFolder()
        {
            var folderPicker = new FolderPicker();
            folderPicker.FileTypeFilter.Add("*");
            var hwnd = ((MauiWinUIWindow)App.Current.Windows[0].Handler.PlatformView).WindowHandle;
            WinRT.Interop.InitializeWithWindow.Initialize(folderPicker, hwnd);

            var result = await folderPicker.PickSingleFolderAsync();

            return result?.Path;
        }

        #endregion

    }

}
