using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

using Prism.Commands;
using Prism.Navigation;

using Demo.PInvoke;
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

            try
            {
                var ret = NativeMethods.cpuinfo_initialize();
                if (!ret)
                {
                    this.Message = "Failed to invoke cpuinfo_initialize";
                    return;
                }

                var count = NativeMethods.cpuinfo_get_packages_count();

                var names = new List<string>();
                for (var index = 0; index < count; index++)
                {
                    var package = NativeMethods.cpuinfo_get_package((uint)index);
                    if (package == IntPtr.Zero)
                        continue;

                    // name member is top of cpuinfo_package, so need not to take care of cpuinfo_package struct
                    var str = Marshal.PtrToStringAnsi(package);
                    names.Add(str);
                }

                this.Message = string.Join("\n", names);
            }
            catch(Exception e)
            {
                this.Message = "Failed to read cpu package names";
            }
        }

        #endregion

        #region Properties

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.NavigationService.NavigateAsync("ShowLogsPage");
                }));
            }
        }

        private string _Message;

        public string Message
        {
            get => this._Message;
            private set => this.SetProperty(ref this._Message, value);
        }

        #endregion

    }

}