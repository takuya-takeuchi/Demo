using System.Windows;
using System.Windows.Media;

namespace WPF4.Controls
{

    public sealed class Range : FrameworkElement
    {

        #region Properties

        public int Position
        {
            get;
            set;
        }

        public int Length
        {
            get;
            set;
        }

        public Brush Brush
        {
            get;
            set;
        }

        public Pen BorderPen
        {
            get;
            set;
        }

        #endregion

    }

}
