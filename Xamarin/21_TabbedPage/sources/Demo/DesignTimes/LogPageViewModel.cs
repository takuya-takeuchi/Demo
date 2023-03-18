using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class LogPageViewModel : ILogPageViewModel
    {

        public string Title
        {
            get;
        }

        public string Logs
        {
            get;
        }

    }

}