using System.Linq;
using System.Windows;
using System.Windows.Controls;
using CNTK6.ViewModels;
using MetroRadiance.UI.Controls;

namespace CNTK6
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : MetroWindow
    {

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            Closing += (s, e) => ViewModelLocator.Cleanup();
        }

        #region プロパティ

        public MainViewModel ViewModel
        {
            get
            {
                return this.DataContext as MainViewModel;
            }
        }

        #endregion

        #region メソッド

        #region イベント

        private void ImageDrop(object sender, DragEventArgs e)
        {
            var viewModel = this.ViewModel;
            if (viewModel == null)
            {
                return;
            }

            var files = e.Data.GetData(DataFormats.FileDrop) as string[];
            if (files == null)
            {
                return;
            }

            viewModel.ImagePath = files.First();
        }

        private void ImagePreviewDragOver(object sender, DragEventArgs e)
        {
            if (!e.Data.GetDataPresent(DataFormats.FileDrop, true))
            {
                e.Effects = DragDropEffects.None;
                return;
            }

            var files = e.Data.GetData(DataFormats.FileDrop) as string[];
            if (files == null || files.Length > 1)
            {
                e.Effects = DragDropEffects.None;
                return;
            }
            
            e.Effects = DragDropEffects.Copy;
            e.Handled = true;
        }

        private void ClickHeader(object sender, RoutedEventArgs e)
        {
            var viewModel = this.ViewModel;
            if (viewModel == null)
            {
                return;
            }

            var header = sender as GridViewColumnHeader;
            if (header == null)
            {
                return;
            }

            var sortBy = header.Content as string;
            if (string.IsNullOrWhiteSpace(sortBy))
            {
                return;
            }

            switch (sortBy)
            {
                case "Label":
                    sortBy = "Label";
                    break;
                case "ProbabilityText":
                    sortBy = "Probability";
                    break;
            }

            viewModel.Sort(sortBy);
        }

        #endregion

        #endregion

    }
}