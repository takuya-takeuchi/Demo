using System.Collections.Generic;

using Xamarin.Forms;

using Prism.Commands;
using Prism.Navigation;

using Sharpnado.MaterialFrame;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Main Page";
        }

        #endregion

        #region Properties
        
        private Command<string> _BlurStyleChangedCommand;

        public Command<string> BlurStyleChangedCommand
        {
            get
            {
                return this._BlurStyleChangedCommand ?? (this._BlurStyleChangedCommand = new Command<string>(buttonName =>
                {
                    this.LoggingService.Info($"Invoked {nameof(BlurStyleChangedCommand)}");

                    MaterialFrame.BlurStyle blurStyle;
                    switch (buttonName)
                    {
                        case "Light":
                            blurStyle = MaterialFrame.BlurStyle.Light;
                            break;
                        case "Dark":
                            blurStyle = MaterialFrame.BlurStyle.Dark;
                            break;
                        default:
                            blurStyle = MaterialFrame.BlurStyle.ExtraLight;
                            break;
                    }

                    ResourcesHelper.SetBlurStyle(blurStyle);

                    this._BlurButtonTextColors[buttonName] = blurStyle != MaterialFrame.BlurStyle.ExtraLight
                        ? ResourcesHelper.GetResourceColor("TextPrimaryColor")
                        : ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");

                    this._BlurButtonBackgroundColors[buttonName] = ResourcesHelper.GetResourceColor(ResourcesHelper.DynamicPrimaryColor);

                    var colors = new[]
                    {
                        "Light", "Dark", "ExtraLight"
                    };

                    foreach (var color in colors)
                    {
                        if (buttonName == color)
                            continue;

                        this._BlurButtonTextColors[color] = blurStyle != MaterialFrame.BlurStyle.ExtraLight
                            ? ResourcesHelper.GetResourceColor("TextPrimaryDarkColor")
                            : ResourcesHelper.GetResourceColor("TextPrimaryColor");
                        this._BlurButtonBackgroundColors[color] = Color.Transparent;
                    }
                }));
            }
        }

        private bool _IsBlurStyleEnabled;

        public bool IsBlurStyleEnabled
        {
            get => this._IsBlurStyleEnabled;
            private set => this.SetProperty(ref this._IsBlurStyleEnabled, value);
        }

        private readonly Dictionary<string, Color> _BlurButtonBackgroundColors = new Dictionary<string, Color>();

        public IDictionary<string, Color> BlurButtonBackgroundColors => this._BlurButtonBackgroundColors;

        private readonly Dictionary<string, Color> _BlurButtonTextColors = new Dictionary<string, Color>();

        public IDictionary<string, Color> BlurButtonTextColors => this._BlurButtonTextColors;

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.LoggingService.Info($"Invoked {nameof(ShowLogCommand)}");

                    this.NavigationService.NavigateAsync("ShowLogs");
                }));
            }
        }

        private Command<bool> _StyleChangedCommand;

        public Command<bool> StyleChangedCommand
        {
            get
            {
                return this._StyleChangedCommand ?? (this._StyleChangedCommand = new Command<bool>(enabled =>
                {
                    this.LoggingService.Info($"Invoked {nameof(StyleChangedCommand)}");

                    if (enabled)
                    {
                        ResourcesHelper.SetAcrylic(true);
                        this.BlurStyleChangedCommand.Execute("Light");
                    }
                    else
                    {
                        ResourcesHelper.SetAcrylic(false);
                    }
                    
                    this.IsBlurStyleEnabled = enabled;
                }));
            }
        }

        #endregion

        #region Methods

        public override void Initialize(INavigationParameters parameters)
        {
            this.LoggingService.Info($"Invoked {nameof(Initialize)}");

            base.Initialize(parameters);

            // Important
            this.IsBlurStyleEnabled = true;
            this.StyleChangedCommand.Execute(false);

            var colors = new[]
            {
                "Light", "Dark", "ExtraLight"
            };

            var buttonName = "Light";
            var blurStyle = MaterialFrame.BlurStyle.Light;
            foreach (var color in colors)
            {
                if (buttonName == color)
                {
                    this._BlurButtonTextColors[color] = ResourcesHelper.GetResourceColor("TextPrimaryColor");
                    this._BlurButtonBackgroundColors[color] = ResourcesHelper.GetResourceColor(ResourcesHelper.DynamicPrimaryColor);
                }
                else
                {
                    this._BlurButtonTextColors[color] = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                    this._BlurButtonBackgroundColors[color] = Color.Transparent;
                }
            }
        }

        #endregion

    }

}