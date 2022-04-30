using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace WPF1.Controls
{

    /// <summary>
    /// このカスタム コントロールを XAML ファイルで使用するには、手順 1a または 1b の後、手順 2 に従います。
    ///
    /// 手順 1a) 現在のプロジェクトに存在する XAML ファイルでこのカスタム コントロールを使用する場合
    /// この XmlNamespace 属性を使用場所であるマークアップ ファイルのルート要素に
    /// 追加します:
    ///
    ///     xmlns:MyNamespace="clr-namespace:WPF1.Controls"
    ///
    ///
    /// 手順 1b) 異なるプロジェクトに存在する XAML ファイルでこのカスタム コントロールを使用する場合
    /// この XmlNamespace 属性を使用場所であるマークアップ ファイルのルート要素に
    /// 追加します:
    ///
    ///     xmlns:MyNamespace="clr-namespace:WPF1.Controls;assembly=WPF1.Controls"
    ///
    /// また、XAML ファイルのあるプロジェクトからこのプロジェクトへのプロジェクト参照を追加し、
    /// リビルドして、コンパイル エラーを防ぐ必要があります:
    ///
    ///     ソリューション エクスプローラーで対象のプロジェクトを右クリックし、
    ///     [参照の追加] の [プロジェクト] を選択してから、このプロジェクトを参照し、選択します。
    ///
    ///
    /// 手順 2)
    /// コントロールを XAML ファイルで使用します。
    ///
    ///     <MyNamespace:GridControl/>
    ///
    /// </summary>
    public class GridControl : Control
    {

        #region フィールド
        #endregion

        #region コンストラクタ

        static GridControl()
        {
            DefaultStyleKeyProperty.OverrideMetadata(typeof(GridControl), new FrameworkPropertyMetadata(typeof(GridControl)));
        }

        public GridControl()
        {
            this.BorderSize = 1;
            this.CellStates = new bool[3,3];
        }

        #endregion

        #region 依存関係プロパティ

        public static readonly DependencyProperty CellStatesProperty =
            DependencyProperty.Register(
                "CellStates",
                typeof(bool[,]),
                typeof(Control),
                new FrameworkPropertyMetadata(
                    null,
                    FrameworkPropertyMetadataOptions.AffectsRender,
                    PropertyChangedCallback));

        public bool[,] CellStates
        {
            get { return (bool[,])GetValue(CellStatesProperty); }
            set { SetValue(CellStatesProperty, value); }
        }

        public static readonly DependencyProperty BorderSizeProperty =
            DependencyProperty.Register(
                "BorderSize",
                typeof(int),
                typeof(Control),
                new FrameworkPropertyMetadata(
                    1,
                    FrameworkPropertyMetadataOptions.AffectsRender,
                    PropertyChangedCallback));

        public int BorderSize
        {
            get { return (int)GetValue(BorderSizeProperty); }
            set { SetValue(BorderSizeProperty, value); }
        }

        #endregion

        #region メソッド

        #region オーバーライド

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);

            var state = this.CellStates;
            if (state == null)
                return;

            // 縦横の次元数
            var v = state.GetLength(0);
            var h = state.GetLength(1);

            if (v == 0 || h == 0)
                return;

            // アンチエイリアスによって線によってサイズが変わるのを防ぐ
            this.VisualEdgeMode = EdgeMode.Unspecified;

            var width = this.ActualWidth;
            var height = this.ActualHeight;
            var borderSize = this.BorderSize;

            // 横、縦線の本数
            var hBorderCount = v + 1;
            var vBorderCount = h + 1;

            // 各セルの幅、高さ
            var hSize = (width - (vBorderCount * borderSize)) / h;
            var vSize = (height - (hBorderCount * borderSize)) / v;

            // 線のブラシ
            var linePen = new Pen(Brushes.Black, borderSize);

            // ペンの中心だけずらす
            var borderGap = borderSize / 2d;

            // 線を描画 (外枠以外)
            hSize += borderSize; // 横方向の座標移動量
            vSize += borderSize; // 縦方向の座標移動量

            var vEnd = height - vSize;
            var hEnd = width - hSize;

            for (var y = vSize + borderGap; y < vEnd; y += vSize)
            {
                for (var x = hSize + borderGap; x < hEnd; x += hSize)
                {
                    // 縦方向
                    drawingContext.DrawLine(
                        linePen,
                        new Point(x, 0),
                        new Point(x, height - borderSize));
                }

                // 横方向
                drawingContext.DrawLine(
                    linePen,
                    new Point(0, y),
                    new Point(width - borderSize, y));
            }

            // 外枠
            drawingContext.DrawRectangle(
                null,
                linePen,
                new Rect(borderGap, borderGap, width - borderSize, height - borderSize));
        }

        #endregion

        #region イベントハンドラ
        #endregion

        #region ヘルパーメソッド

        private static void PropertyChangedCallback(DependencyObject dependencyObject, DependencyPropertyChangedEventArgs dependencyPropertyChangedEventArgs)
        {
            //throw new NotImplementedException();
        }

        #endregion

        #endregion


    }

}
