using System;
using System.Windows;
using System.Windows.Interop;
namespace Wpf
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += this.WindowLoaded;
        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            var source = HwndSource.FromHwnd(new WindowInteropHelper(this).Handle);
            source?.AddHook(WndProc);
        }

        private static IntPtr WndProc(IntPtr hwnd, int msg, IntPtr wParam, IntPtr lParam, ref bool handled)
        {
            const int WM_CLOSE = 0x0010;
            if (msg == WM_CLOSE)
            {
                MessageBox.Show("Get WM_CLOSE");

                //handled = true;
            }

            return IntPtr.Zero;
        }

    }
}
