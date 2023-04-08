using Xamarin.Forms;

using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public Command<string> BlurStyleChangedCommand
        {
            get;
        }

        public Color EntryTextColor
        {
            get;
        }

        public Color ExtraLightButtonTextColor
        {
            get;
        }

        public Color ExtraLightButtonBackgroundColor
        {
            get;
        }

        public Color LightButtonTextColor
        {
            get;
        }

        public Color LightButtonBackgroundColor
        {
            get;
        }

        public Color DarkButtonTextColor
        {
            get;
        }

        public Color DarkButtonBackgroundColor
        {
            get;
        }

        public bool IsBlurStyleEnabled
        {
            get;
        }

        public DelegateCommand ShowLogCommand
        {
            get;
        }

        public Command<bool> StyleChangedCommand
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
