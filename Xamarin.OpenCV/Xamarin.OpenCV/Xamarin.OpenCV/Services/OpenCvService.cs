namespace Xamarin.OpenCV.Services
{

    public sealed class OpenCvService : IOpenCVService
    {

        public string GetBuildInformation()
        {
            return OpenCV.GetBuildInformation();
        }

        public string GetVersion()
        {
            return OpenCV.GetVersion();
        }

    }

}