using System.Collections.Generic;

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

        IDictionary<string, Color> BlurButtonBackgroundColors
        {
            get;
        }

        IDictionary<string, Color> BlurButtonTextColors
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