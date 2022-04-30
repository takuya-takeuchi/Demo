using System;
using System.Net.Http;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPFPython.Models;
using WPFPython.ViewModels.Interfaces;

namespace WPFPython.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region フィールド

        private readonly IMessageDialog _MessageDialog;

        #endregion

        #region コンストラクタ

        public MainViewModel(IMessageDialog messageDialog)
        {
            if (messageDialog == null)
                throw new ArgumentNullException(nameof(messageDialog));

            this._MessageDialog = messageDialog;
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
                    var stringAsync = httpClient.GetStringAsync("http://127.0.0.1:5000/");
                    string result = await stringAsync;

                    var messageModel = Newtonsoft.Json.JsonConvert.DeserializeObject<MessageModel>(result);
                    this._MessageDialog.ShowMessage(messageModel.Hello);
                });
            }
        }

        #endregion

    }
}
