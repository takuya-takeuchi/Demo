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
            this.Text = "38520000023237";
        }

        #endregion

        #region IMainPageViewModel Members

        private string _Text = "";

        public string Text
        {
            get => this._Text;
            set 
            {
                this.SetProperty(ref this._Text, value);

                try
                {
                    var str = System.Text.Encoding.UTF8.GetBytes(this.Text + "\0");
                    //var ret = NativeMethods.luhn_validateString(str, 1);
                    var type = NativeMethods.luhn_typeFromString(str) - 1;

                    //var types = new []
                    //{
                    //    "Amex",
                    //    "Visa",
                    //    "Mastercard",
                    //    "Discover",
                    //    "DinersClub",
                    //    "JCB",
                    //    "Unsupported",
                    //    "Invalid"                        
                    //};

                    //this.CardType = types[type];
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }
            }
        }

        private string _CardType;

        public string CardType
        {
            get => this._CardType;
            private set => this.SetProperty(ref this._CardType, value);
        }

        #endregion

    }

}