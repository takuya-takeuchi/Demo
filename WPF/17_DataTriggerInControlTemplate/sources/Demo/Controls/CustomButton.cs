using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace Demo.Controls
{

    public sealed class CustomButton : Button
    {

        #region Constructors

        static CustomButton()
        {
            DefaultStyleKeyProperty.OverrideMetadata(typeof(CustomButton), new FrameworkPropertyMetadata(typeof(CustomButton)));
        }

        #endregion

        #region Dependency Properties

        #region Checked

        public static readonly DependencyProperty CheckedProperty =
            DependencyProperty.Register(nameof(Checked),
                typeof(bool),
                typeof(CustomButton),
                new UIPropertyMetadata(false));

        public bool Checked
        {
            get => (bool)this.GetValue(CheckedProperty);
            set => this.SetValue(CheckedProperty, value);
        }

        #endregion

        #region CheckedForeground

        public static readonly DependencyProperty CheckedForegroundProperty =
            DependencyProperty.Register(nameof(CheckedForeground),
                typeof(Brush),
                typeof(CustomButton),
                new UIPropertyMetadata(null));

        public Brush CheckedForeground
        {
            get => (Brush)this.GetValue(CheckedForegroundProperty);
            set => this.SetValue(CheckedForegroundProperty, value);
        }

        #endregion

        #region UncheckedForeground

        public static readonly DependencyProperty UncheckedForegroundProperty =
            DependencyProperty.Register(nameof(UncheckedForeground),
                typeof(Brush),
                typeof(CustomButton),
                new UIPropertyMetadata(null));

        public Brush UncheckedForeground
        {
            get => (Brush)this.GetValue(UncheckedForegroundProperty);
            set => this.SetValue(UncheckedForegroundProperty, value);
        }

        #endregion

        #endregion

    }

}
