using System;
using System.Windows.Input;
using Microsoft.Practices.Prism.Mvvm;

namespace Xamarin.Forms.Portable2.ViewModels
{
    public sealed class MainPageViewModel : BindableBase
    {

        public MainPageViewModel()
        {
            this.IsRunning = false;
            this.TimeSpan = new TimeSpan(0, 0, 0, 0, 0);
        }

        private TimeSpan _TimeSpan;

        public TimeSpan TimeSpan
        {
            get
            {
                return this._TimeSpan;
            }
            private set
            {

                this.Minute = this.TimeSpan.Minutes.ToString("00");
                this.Second = this.TimeSpan.Seconds.ToString("00");

                var ms = this.TimeSpan.Milliseconds / 10;
                this.Millisecond = ms.ToString("00");
                this.SetProperty(ref this._TimeSpan, value);
            }
        }

        private string _Minute;

        public string Minute
        {
            get
            {
                return this._Minute;
            }
            private set
            {
                this.SetProperty(ref this._Minute, value);
            }
        }

        private string _Second;

        public string Second
        {
            get
            {
                return this._Second;
            }
            private set
            {
                this.SetProperty(ref this._Second, value);
            }
        }

        private string _Millisecond;

        public string Millisecond
        {
            get
            {
                return this._Millisecond;
            }
            private set
            {
                this.SetProperty(ref this._Millisecond, value);
            }
        }

        private bool _IsRunning = false;

        public bool IsRunning
        {
            get
            {
                return this._IsRunning;
            }
            private set
            {
                this.SetProperty(ref this._IsRunning, value);
            }
        }

        private ICommand _StartCommand;

        public ICommand StartCommand
        {
            get
            {
                if (this._StartCommand == null)
                {
                    this._StartCommand = new Command(() =>
                    {
                        this.IsRunning = true;

                        Device.StartTimer(new TimeSpan(0, 0, 0, 0, 10),
                            () =>
                            {
                                this.TimeSpan = this.TimeSpan.Add(new TimeSpan(0, 0, 0, 0, 10));
                                return this.IsRunning;
                            });
                    });
                }

                return this._StartCommand;
            }
        }

        private ICommand _StopCommand;

        public ICommand StopCommand
        {
            get
            {
                if (this._StopCommand == null)
                {
                    this._StopCommand = new Command(() =>
                    {
                        this.IsRunning = false;
                    });
                }

                return this._StopCommand;
            }
        }

    }

}
