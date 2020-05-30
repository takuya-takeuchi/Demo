using Xamarin.OpenCV.Services.Interfaces;

namespace Xamarin.OpenCV.Services.Mocks
{

    public sealed class OpenCvService : IOpenCVService
    {

        public string GetBuildInformation()
        {
            throw new System.NotImplementedException();
        }

        public string GetVersion()
        {
            return "4.3.0";
        }

    }

}