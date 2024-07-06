using System.Threading.Tasks;

namespace Demo.Services.Interfaces
{

    internal interface IDialogService
    {

        Task ShowAlert(string title, string message, string cancel = "OK");

    }

}
