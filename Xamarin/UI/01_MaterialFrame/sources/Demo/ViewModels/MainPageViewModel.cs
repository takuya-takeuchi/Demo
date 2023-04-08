using Xamarin.Forms;

using Prism.Commands;
using Prism.Navigation;

using Sharpnado.MaterialFrame;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : PageViewModelBase, IMainPageViewModel
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
                            ResourcesHelper.SetBlurStyle(MaterialFrame.BlurStyle.Light);
                            this.EntryTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.LightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryColor");
                            this.LightButtonBackgroundColor = ResourcesHelper.GetResourceColor(ResourcesHelper.DynamicPrimaryColor);

                            this.DarkButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.DarkButtonBackgroundColor = Color.Transparent;
                            this.ExtraLightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.ExtraLightButtonBackgroundColor = Color.Transparent;
                            break;
                        case "Dark":
                            ResourcesHelper.SetBlurStyle(MaterialFrame.BlurStyle.Dark);
                            this.EntryTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.DarkButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryColor");
                            this.DarkButtonBackgroundColor = ResourcesHelper.GetResourceColor(ResourcesHelper.DynamicPrimaryColor);

                            this.LightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.LightButtonBackgroundColor = Color.Transparent;
                            this.ExtraLightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.ExtraLightButtonBackgroundColor = Color.Transparent;
                            break;
                        default:
                            ResourcesHelper.SetBlurStyle(MaterialFrame.BlurStyle.ExtraLight);
                            this.EntryTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.ExtraLightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryDarkColor");
                            this.ExtraLightButtonBackgroundColor = ResourcesHelper.GetResourceColor(ResourcesHelper.DynamicPrimaryColor);

                            this.LightButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryColor");
                            this.LightButtonBackgroundColor = Color.Transparent;
                            this.DarkButtonTextColor = ResourcesHelper.GetResourceColor("TextPrimaryColor");
                            this.DarkButtonBackgroundColor = Color.Transparent;
                            break;
                    }
                }));
            }
        }

        private Color _EntryTextColor;

        public Color EntryTextColor
        {
            get => this._EntryTextColor;
            private set => this.SetProperty(ref this._EntryTextColor, value);
        }

        private Color _ExtraLightButtonTextColor;

        public Color ExtraLightButtonTextColor
        {
            get => this._ExtraLightButtonTextColor;
            private set => this.SetProperty(ref this._ExtraLightButtonTextColor, value);
        }

        private Color _ExtraLightButtonBackgroundColor;

        public Color ExtraLightButtonBackgroundColor
        {
            get => this._ExtraLightButtonBackgroundColor;
            private set => this.SetProperty(ref this._ExtraLightButtonBackgroundColor, value);
        }

        private Color _LightButtonTextColor;

        public Color LightButtonTextColor
        {
            get => this._LightButtonTextColor;
            private set => this.SetProperty(ref this._LightButtonTextColor, value);
        }

        private Color _LightButtonBackgroundColor;

        public Color LightButtonBackgroundColor
        {
            get => this._LightButtonBackgroundColor;
            private set => this.SetProperty(ref this._LightButtonBackgroundColor, value);
        }

        private Color _DarkButtonTextColor;

        public Color DarkButtonTextColor
        {
            get => this._DarkButtonTextColor;
            private set => this.SetProperty(ref this._DarkButtonTextColor, value);
        }

        private Color _DarkButtonBackgroundColor;

        public Color DarkButtonBackgroundColor
        {
            get => this._DarkButtonBackgroundColor;
            private set => this.SetProperty(ref this._DarkButtonBackgroundColor, value);
        }

        private bool _IsBlurStyleEnabled;

        public bool IsBlurStyleEnabled
        {
            get => this._IsBlurStyleEnabled;
            private set => this.SetProperty(ref this._IsBlurStyleEnabled, value);
        }

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.LoggingService.Info($"Invoked {nameof(ShowLogCommand)}");

                    this.NavigationService.NavigateAsync("ShowLogsPage");
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
        }

        #endregion

    }

}