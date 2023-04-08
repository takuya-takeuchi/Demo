using Xamarin.Forms;

using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        Command<string> BlurStyleChangedCommand
        {
            get;
        }

        Color EntryTextColor
        {
            get;
        }

        Color ExtraLightButtonTextColor
        {
            get;
        }

        Color ExtraLightButtonBackgroundColor
        {
            get;
        }

        Color LightButtonTextColor
        {
            get;
        }

        Color LightButtonBackgroundColor
        {
            get;
        }

        Color DarkButtonTextColor
        {
            get;
        }

        Color DarkButtonBackgroundColor
        {
            get;
        }

        bool IsBlurStyleEnabled
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        Command<bool> StyleChangedCommand
        {
            get;
        }

    }

}