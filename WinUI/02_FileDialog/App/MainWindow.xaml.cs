using System;
using Windows.Storage.Pickers;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media.Imaging;
using WinRT.Interop;

// To learn more about WinUI, the WinUI project structure,
// and more about our project templates, see: http://aka.ms/winui-project-info.

namespace App
{

    /// <summary>
    /// An empty window that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainWindow : Window
    {

        #region Construtors

        public MainWindow()
        {
            this.InitializeComponent();
        }

        #endregion

        #region Event Handlers

        private async void myButton_Click(object sender, RoutedEventArgs e)
        {
            var hwnd = WindowNative.GetWindowHandle(this);

            try
            {
                var picker = new FileOpenPicker();
                picker.FileTypeFilter.Add(".jpg");

                // Require
                // using WinRT.Interop;
                InitializeWithWindow.Initialize(picker, hwnd);

                var file = await picker.PickSingleFileAsync();
                if (string.IsNullOrWhiteSpace(file?.Path))
                    return;

                var cd = new ContentDialog
                {
                    Title = "Open File Dialog",
                    Content = new Image
                    {
                        Source = new BitmapImage
                        {
                            UriSource = new Uri(file.Path)
                        }
                    },
                    CloseButtonText = "OK"
                };

                cd.XamlRoot = this.Content.XamlRoot;
                await cd.ShowAsync();
            }
            catch (Exception exception)
            {
                Console.WriteLine(exception);
            }
        }

        #endregion

    }

}
