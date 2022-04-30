using System;
using System.Collections.ObjectModel;
using System.Net.Http;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPFPython.Contracts.Service;
using WPFPython.ViewModels.Interfaces;

namespace WPFPython.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region フィールド

        private readonly IDateTimeService _DateTimeService;

        #endregion

        #region コンストラクタ

        public MainViewModel(IDateTimeService dateTimeService)
        {
            if (dateTimeService == null)
                throw new ArgumentNullException(nameof(dateTimeService));

            this._DateTimeService = dateTimeService;
            this._DateTimeService.DateTimeReceived += (sender, time) =>
            {
                this._Responses.Insert(0, time.ToString("yyyy/MM/dd hh:mm:ss.fff"));
            };

            this._Responses = new ObservableCollection<string>();
        }

        #endregion

        #region プロパティ

        private RelayCommand _MessageCommand;

        public RelayCommand MessageCommand
        {
            get
            {
                return this._MessageCommand ?? new RelayCommand(async () =>
                {
                    var httpClient = new HttpClient();
                    await httpClient.GetStringAsync("http://127.0.0.1:5000/");
                });
            }
        }

        private readonly ObservableCollection<string> _Responses;

        public ObservableCollection<string> Responses
        {
            get
            {
                return this._Responses;
            }
        }

        #endregion

    }

}
