using System;
using System.Threading.Tasks;

using Demo.Services.Interfaces;

namespace Demo.Platforms.Android.Services
{

    internal sealed class FolderPickerService : IFolderPickerService
    {

        #region IFolderPickerService Members

        public Task<string> PickFolder()
        {
            throw new NotImplementedException();
        }

        #endregion

    }

}
