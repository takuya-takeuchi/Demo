using System;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using GalaSoft.MvvmLight;
using Image3.ViewModels.Interfaces;

namespace Image3.ViewModels
{
    /// <summary>
    /// This class contains properties that the main View can data bind to.
    /// <para>
    /// Use the <strong>mvvminpc</strong> snippet to add bindable properties to this ViewModel.
    /// </para>
    /// <para>
    /// You can also use Blend to data bind with the tool's support.
    /// </para>
    /// <para>
    /// See http://www.galasoft.ch/mvvm
    /// </para>
    /// </summary>
    public class MainViewModel : ViewModelBase, IMainViewModel
    {


        #region イベント
        #endregion

        #region フィールド

        private BitmapSource _FilteredSource;

        private BitmapSource _OriginalSource;

        private int _MaxHue;

        private double _MaxSaturation;

        private double _MaxValue;

        private int _MinHue;

        private double _MinValue;

        private double _MinSaturation;

        private readonly byte[] _ImageBuffer;

        private readonly byte[] _FilteredImageBuffer;

        private readonly int _Width;

        private readonly int _Height;

        private readonly int _Stride;

        private readonly int _Channel;

        private readonly PixelFormat _PixelFormat;

        private bool _BackColor;

        #endregion

        #region コンストラクタ

        public MainViewModel()
        {
            this.MaxHue = 360;
            this.MinHue = 0;
            this.MaxSaturation = 1.0d;
            this.MinSaturation = 0.5d;
            this.MaxValue = 1.0d;
            this.MinValue = 0.5d;
            this.BackColor = true;

            var bmp = new BitmapImage(
                new Uri("Resources/colorful-cats-wallpaper-background.jpg", UriKind.RelativeOrAbsolute));

            // 生のデータ配列を複製
            int channel = bmp.Format.BitsPerPixel / 8;
            int width = bmp.PixelWidth;
            int stride = (width + ((width % 4 == 0) ? 0 : 4 - (width % 4))) * channel;
            int height = bmp.PixelHeight;
            var offset = stride / channel - width;

            this._Width = width;
            this._Height = height;
            this._Stride = stride;
            this._Channel = channel;
            this._PixelFormat = bmp.Format;

            this._ImageBuffer = new byte[(width+ offset) * height * channel];
            this._FilteredImageBuffer = new byte[(width + offset) * height * channel];
            bmp.CopyPixels(this._ImageBuffer, stride, 0);
            bmp.Freeze();
            this.OriginalSource = bmp;

            this.PropertyChanged += (sender, args) =>
            {
                switch (args.PropertyName)
                {
                    case nameof(this.MaxHue):
                    case nameof(this.MinHue):
                    case nameof(this.MaxSaturation):
                    case nameof(this.MinSaturation):
                    case nameof(this.MaxValue):
                    case nameof(this.MinValue):
                    case nameof(this.BackColor):
                        this.UpdateImage();
                        break;
                }
            };

            this.UpdateImage();
        }

        #endregion

        #region プロパティ

        public BitmapSource OriginalSource
        {
            get
            {
                return this._OriginalSource;
            }
            private set
            {
                this._OriginalSource = value;
                this.RaisePropertyChanged();
            }
        }

        public BitmapSource FilteredSource
        {
            get
            {
                return this._FilteredSource;
            }
            private set
            {
                this._FilteredSource = value;
                this.RaisePropertyChanged();
            }
        }

        public int MaxHue
        {
            get
            {
                return this._MaxHue;
            }
            set
            {
                this._MaxHue = value;
                this.RaisePropertyChanged();
            }
        }

        public double MaxSaturation
        {
            get
            {
                return this._MaxSaturation;
            }
            set
            {
                this._MaxSaturation = value;
                this.RaisePropertyChanged();
            }
        }

        public double MaxValue
        {
            get
            {
                return this._MaxValue;
            }
            set
            {
                this._MaxValue = value;
                this.RaisePropertyChanged();
            }
        }

        public int MinHue
        {
            get
            {
                return this._MinHue;
            }
            set
            {
                this._MinHue = value;
                this.RaisePropertyChanged();
            }
        }

        public double MinSaturation
        {
            get
            {
                return this._MinSaturation;
            }
            set
            {
                this._MinSaturation = value;
                this.RaisePropertyChanged();
            }
        }

        public double MinValue
        {
            get
            {
                return this._MinValue;
            }
            set
            {
                this._MinValue = value;
                this.RaisePropertyChanged();
            }
        }

        public bool BackColor
        {
            get
            {
                return this._BackColor;
            }
            set
            {
                this._BackColor = value;
                this.RaisePropertyChanged();
            }
        }

        #endregion

        #region メソッド

        private void UpdateImage()
        {
            unsafe
            {
                fixed (byte* pSrc = &this._ImageBuffer[0])
                fixed (byte* pDst = &this._FilteredImageBuffer[0])
                {
                    var ps = pSrc;
                    var pd = pDst;

                    var stride = this._Stride;
                    var width = this._Width;
                    var height = this._Height;
                    var channel = this._Channel;
                    var dpiX = this._OriginalSource.DpiX;
                    var dpiY = this._OriginalSource.DpiY;
                    var offset = stride / channel - width;

                    var maxHue = this._MaxHue;
                    var minHue = this._MinHue;
                    var maxSaturation = this._MaxSaturation;
                    var minSaturation = this._MinSaturation;
                    var maxValue = this._MaxValue;
                    var minValue = this._MinValue;

                    float h, s, v;
                    var back = (byte)(this._BackColor ? 0 : 255);
                    for (var y = 0; y < height; y++)
                    {
                        for (var x = 0; x < width; x++, pd += channel, ps += channel)
                        {
                            FromRgb(ps[2], ps[1], ps[0], out h, out s, out v);
                            if (minHue <= h && h <= maxHue &&
                                minSaturation <= s && s <= maxSaturation &&
                                minValue <= v && v <= maxValue)
                            {
                                pd[2] = ps[2];
                                pd[1] = ps[1];
                                pd[0] = ps[0];
                            }
                            else
                            {
                                // 白にする
                                pd[2] = back;
                                pd[1] = back;
                                pd[0] = back;
                            }
                        }

                        pd += offset;
                        ps += offset;
                    }
                    
                    var bitmap = BitmapSource.Create(
                        width,
                        height,
                        dpiX,
                        dpiY,
                        this._PixelFormat, 
                        null, 
                        this._FilteredImageBuffer,
                        stride);
                    bitmap.Freeze();
                    this.FilteredSource = bitmap;
                }
            }
        }

        private static void FromRgb(byte r, byte g, byte b, out float h, out float s, out float v)
        {
            var max = Math.Max(r, Math.Max(g, b));
            var min = Math.Min(r, Math.Min(g, b));

            var brightness = max / 255f;
            float hue, saturation;
            if (Math.Abs(max - min) < float.Epsilon)
            {
                hue = 0f;
                saturation = 0f;
            }
            else
            {
                float c = max - min;
                if (Math.Abs(max - r) < float.Epsilon)
                    hue = (g - b) / c;
                else if (Math.Abs(max - g) < float.Epsilon)
                    hue = (b - r) / c + 2f;
                else
                    hue = (r - g) / c + 4f;

                hue *= 60f;
                if (hue < 0f)
                    hue += 360f;

                saturation = c / max;
            }

            h = hue;
            s = saturation;
            v = brightness;
        }

        #endregion

    }

}

namespace Image3.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        BitmapSource OriginalSource
        {
            get;
        }


        BitmapSource FilteredSource
        {
            get;
        }

        int MaxHue
        {
            get;
            set;
        }

        double MaxSaturation
        {
            get;
            set;
        }

        double MaxValue
        {
            get;
            set;
        }

        int MinHue
        {
            get;
            set;
        }

        double MinSaturation
        {
            get;
            set;
        }

        double MinValue
        {
            get;
            set;
        }

        bool BackColor
        {
            get;
            set;
        }

    }
}