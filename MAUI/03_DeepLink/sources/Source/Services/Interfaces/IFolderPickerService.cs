using System.Threading.Tasks;

namespace Source.Services.Interfaces
{

    internal interface IFolderPickerService
    {

        Task<string> PickFolder();

    }

}
