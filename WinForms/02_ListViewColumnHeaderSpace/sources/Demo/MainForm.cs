using System.Windows.Forms;

namespace Demo
{

    public partial class MainForm : Form
    {

        #region Constructors

        public MainForm()
        {
            InitializeComponent();

            for (var index = 0; index < 10; index++)
            {
                var column = 1;
                var item = new ListViewItem();
                item.Text = $@"item {column}-{index + 1}";
                item.SubItems.Add($"item {column++}-{index + 1}");
                item.SubItems.Add($"item {column++}-{index + 1}");
                item.SubItems.Add($"item {column++}-{index + 1}");
                this._ListView.Items.Add(item);
            }
        }

        #endregion

    }

}