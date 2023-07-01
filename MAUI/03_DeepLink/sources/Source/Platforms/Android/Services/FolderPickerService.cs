using System;
using System.Threading.Tasks;

using Microsoft.Maui.Storage;

namespace Demo.Platforms.Android.Services
{

    internal sealed class FolderPickerService : IFolderPickerService
    {

        #region IFolderPickerService Members

        public async Task<string> PickFolder()
        {
            var ret = await FilePicker.PickAsync();
            //throw new NotImplementedException();

            return ret?.FullPath;
        }

        #endregion

    }

}
