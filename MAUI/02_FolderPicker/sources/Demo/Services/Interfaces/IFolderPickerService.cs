using System.Threading.Tasks;

namespace Demo.Services.Interfaces
{

    internal interface IFolderPickerService
    {

        Task<string> PickFolder();

    }

}
