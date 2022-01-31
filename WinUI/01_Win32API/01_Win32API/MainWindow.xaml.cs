using System;
using System.Runtime.InteropServices;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using WinRT;

// To learn more about WinUI, the WinUI project structure,
// and more about our project templates, see: http://aka.ms/winui-project-info.

namespace _01_Win32API
{

    /// <summary>
    /// An empty window that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainWindow : Window
    {

        #region P/Invokes

        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        private static extern int GetWindowTextLength(IntPtr hwnd);

        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        private static extern int GetWindowText(IntPtr hWnd, string lpString, int nMaxCount);

        #endregion

        #region Construtors

        public MainWindow()
        {
            this.InitializeComponent();
        }

        #endregion

        #region Event Handlers

        private async void myButton_Click(object sender, RoutedEventArgs e)
        {
            // Require using WinRT; 
            var windowNative = this.As<IWindowNative>();
            var hwnd = windowNative.WindowHandle;

            var length = GetWindowTextLength(hwnd);

            string lpString = new string('0', length + 1);
            var ret = GetWindowText(hwnd, lpString, length + 1);

            var cd = new ContentDialog
            {
                Title = "Hello Win32 API",
                Content = $"GetWindowText returns '{lpString}' [{ret}]",
                CloseButtonText = "OK"
            };

            cd.XamlRoot = this.Content.XamlRoot;
            await cd.ShowAsync();
        }

        #endregion

    }

}
