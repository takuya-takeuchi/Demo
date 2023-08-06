using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using Demo.PInvoke;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private int _Count;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            try
            {
                var exist = System.IO.File.Exists("libcpuinfo.dylib");
                var ret = NativeMethods.cpuinfo_initialize();
                if (!ret)
                {
                    this.Text = "Failed to invoke cpuinfo_initialize";
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

                this.Text = string.Join("\n", names);
            }
            catch(Exception e)
            {
                this.Text = "Failed to read cpu package names";
            }
        }

        #endregion

        #region IMainPageViewModel Members

        private string _Text;

        public string Text
        {
            get => this._Text;
            private set => this.SetProperty(ref this._Text, value);
        }

        #endregion

    }

}