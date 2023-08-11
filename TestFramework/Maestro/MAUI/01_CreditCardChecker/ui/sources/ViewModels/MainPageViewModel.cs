using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using Demo.PInvoke;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            // var exist = System.IO.File.Exists("lib/arm64-v8a/libLuhn.so");
            // exist = System.IO.File.Exists("arm64-v8a/libLuhn.so");
            // exist = System.IO.File.Exists("libLuhn.so");
            // exist = System.IO.File.Exists("lib/arm64-v8a/libmonodroid.so");
            // exist = System.IO.File.Exists("arm64-v8a/libmonodroid.so");
            // exist = System.IO.File.Exists("libmonodroid.so");
            // System.Runtime.InteropServices.NativeLibrary.Load("lib/arm64-v8a/libLuhn.so");
            this.CardNumber = "38520000023237";
        }

        #endregion

        #region IMainPageViewModel Members

        private string _CardNumber = "";

        public string CardNumber
        {
            get => this._CardNumber;
            set 
            {
                this.SetProperty(ref this._CardNumber, value);

                if (string.IsNullOrEmpty(this._CardNumber))
                {
                    this.IsValid = false;
                    return;
                }

                try
                {
                    var str = System.Text.Encoding.ASCII.GetBytes(this._CardNumber);
                    this.IsValid = NativeMethods.luhn_validateString(str, (uint)str.Length);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }
            }
        }

        private bool _IsValid;

        public bool IsValid
        {
            get => this._IsValid;
            private set => this.SetProperty(ref this._IsValid, value);
        }

        #endregion

    }

}