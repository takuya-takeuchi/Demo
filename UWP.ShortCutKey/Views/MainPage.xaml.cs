using Windows.UI.Xaml.Controls;
using Prism.Windows.Mvvm;
using UWP.ShortCutKey.ViewModels;
using UWP.ShortCutKey.ViewModels.Interfaces;

// 空白ページの項目テンプレートについては、https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x411 を参照してください

namespace UWP.ShortCutKey.Views
{
    /// <summary>
    /// それ自体で使用できる空白ページまたはフレーム内に移動できる空白ページ。
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public IMainPageViewModel ViewModel => this.DataContext as IMainPageViewModel;

        public MainPage()
        {
            this.InitializeComponent();
        }
    }
}
