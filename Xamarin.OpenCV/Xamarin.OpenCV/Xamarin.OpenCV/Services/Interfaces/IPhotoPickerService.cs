using System.IO;
using System.Threading.Tasks;

namespace Xamarin.OpenCV.Services
{

    public interface IPhotoPickerService
    {

        Task<Stream> GetImageStreamAsync();

    }

}