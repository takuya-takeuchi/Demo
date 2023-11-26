using System.Windows.Forms;

namespace Demo
{

    public partial class MainForm : Form
    {

        #region Constructors

        public MainForm()
        {
            InitializeComponent();
        }

        #endregion

        #region Methods

        #region Overrides

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            this._labelMessage.Text = keyData.ToString();
            return base.ProcessCmdKey(ref msg, keyData);
        }

        #endregion

        #endregion

    }

}