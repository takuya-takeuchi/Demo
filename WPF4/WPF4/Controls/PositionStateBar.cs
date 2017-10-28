using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace WPF4.Controls
{

    public class PositionStateBar : Control
    {

        #region Constructors

        static PositionStateBar()
        {
            DefaultStyleKeyProperty.OverrideMetadata(typeof(PositionStateBar), new FrameworkPropertyMetadata(typeof(PositionStateBar)));
        }

        public PositionStateBar()
        {
            // NOTE
            // Do NOT set value for StartPosition, EndPostion and Orientation here!!

            this.Ranges = new ObservableCollection<Range>();
        }

        #endregion

        #region Properties

        #region Dependency Properties

        public static readonly DependencyProperty EndPositionProperty =
            DependencyProperty.Register(
                "EndPosition",
                typeof(int),
                typeof(PositionStateBar),
                new FrameworkPropertyMetadata(
                    0,
                    PropertyChangedCallback));

        public int EndPosition
        {
            get => (int)GetValue(EndPositionProperty);
            set => SetValue(EndPositionProperty, value);
        }

        public static readonly DependencyProperty OrientationProperty =
            DependencyProperty.Register(
                "Orientation",
                typeof(Orientation),
                typeof(PositionStateBar),
                new FrameworkPropertyMetadata(
                    Orientation.Horizontal,
                    PropertyChangedCallback));

        public Orientation Orientation
        {
            get => (Orientation)GetValue(OrientationProperty);
            set => SetValue(OrientationProperty, value);
        }

        public static readonly DependencyProperty RangesProperty =
            DependencyProperty.Register(
                "Ranges",
                typeof(ObservableCollection<Range>),
                typeof(PositionStateBar),
                new FrameworkPropertyMetadata(
                    new ObservableCollection<Range>(),
                    PropertyChangedCallback));

        public ObservableCollection<Range> Ranges
        {
            get => (ObservableCollection<Range>)GetValue(RangesProperty);
            set => SetValue(RangesProperty, value);
        }

        public static readonly DependencyProperty StartPositionProperty =
            DependencyProperty.Register(
                "StartPosition",
                typeof(int),
                typeof(PositionStateBar),
                new FrameworkPropertyMetadata(
                    0,
                    PropertyChangedCallback));

        public int StartPosition
        {
            get => (int)GetValue(StartPositionProperty);
            set => SetValue(StartPositionProperty, value);
        }

        #endregion

        #endregion

        #region Methods

        #region Overrids

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);

            var ranges = this.Ranges;
            if (ranges == null)
                return;

            var start = Math.Min(this.StartPosition, this.EndPosition);
            var end = Math.Max(this.StartPosition, this.EndPosition);
            var length = end - start;
            if (length == 0)
                return;

            var width = this.ActualWidth;
            var height = this.ActualHeight;

            switch (this.Orientation)
            {
                case Orientation.Horizontal:
                    {
                        var scale = width / length;
                        foreach (var r in ranges)
                        {
                            var x = r.Position * scale;
                            var w = r.Length * scale;
                            drawingContext.DrawRectangle(
                                r.Brush,
                                r.BorderPen,
                                new Rect(x, 0, w, height));
                        }
                    }
                    break;
                case Orientation.Vertical:
                    {
                        var scale = height / length;
                        foreach (var r in ranges)
                        {
                            var y = r.Position * scale;
                            var h = r.Length * scale;
                            drawingContext.DrawRectangle(
                                r.Brush,
                                r.BorderPen,
                                new Rect(0, y, width, h));
                        }
                    }
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        #endregion

        #region Event Handlers
        #endregion

        #region Helpers

        private static void PropertyChangedCallback(DependencyObject dependencyObject, DependencyPropertyChangedEventArgs dependencyPropertyChangedEventArgs)
        {
        }

        #endregion

        #endregion

    }
}
