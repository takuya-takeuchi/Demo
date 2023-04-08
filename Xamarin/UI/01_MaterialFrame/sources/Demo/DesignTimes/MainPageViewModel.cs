using System.Collections.Generic;
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

        public IDictionary<string, Color> BlurButtonBackgroundColors
        {
            get;
        }

        public IDictionary<string, Color> BlurButtonTextColors
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
