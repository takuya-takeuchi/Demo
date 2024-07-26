using System.Windows.Forms;

using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Microsoft.Toolkit.Forms.UI.XamlHost;

namespace Demo
{

    public partial class MainForm : Form
    {

        #region Fields

        private readonly WindowsXamlHost _XamlHost;

        private readonly CalendarView _CalendarView;

        #endregion

        #region Constructors

        public MainForm()
        {
            InitializeComponent();

            this._XamlHost = new WindowsXamlHost();
            this._XamlHost.Width = 400;
            this._XamlHost.Height = 400;

            this._CalendarView = new CalendarView();
            this._CalendarView.RequestedTheme = ElementTheme.Dark;
            this._CalendarView.Width = 400;
            this._CalendarView.Height = 400;

            this._XamlHost.Child = this._CalendarView;

            this.Controls.Add(this._XamlHost);
        }

        #endregion

    }

}