using System.Threading.Tasks;

using Microsoft.Maui.Controls;

using Demo.Services.Interfaces;

namespace Demo.Services
{

    internal sealed class DialogService : IDialogService
    {

        #region IDialogService Members

        public async Task ShowAlert(string title, string message, string cancel = "OK")
        {
            await Application.Current.MainPage.DisplayAlert(title, message, cancel);
        }

        #endregion

    }

}
