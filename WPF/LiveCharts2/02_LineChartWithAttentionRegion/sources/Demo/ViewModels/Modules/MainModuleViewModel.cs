using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Threading;
using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.Painting;
using LiveChartsCore.SkiaSharpView.VisualElements;
using NLog;
using Prism.Commands;
using Prism.Mvvm;
using Prism.Regions;
using SkiaSharp;

using Demo.ViewModels.Modules.Interfaces;
using System.Windows.Controls.Primitives;
using static System.Net.WebRequestMethods;
using LiveChartsCore.Themes;

namespace Demo.ViewModels.Modules
{

    internal sealed class MainModuleViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        private readonly DispatcherTimer _Timer = new DispatcherTimer(DispatcherPriority.Normal);

        private int _Current = 0;

        private readonly int _XAxisMax = 0;

        #endregion

        #region Constructors

        public MainModuleViewModel(IRegionManager regionManager)
        {
            this._RegionManager = regionManager;
            Logger.Info($"Constructor of {nameof(MainModuleViewModel)}");

            // load chart data
            var uri = new Uri("/Resources/HistoricalData_1721962574697.csv", UriKind.Relative);
            var streamResourceInfo = Application.GetResourceStream(uri);
            var stream = streamResourceInfo.Stream;
            using var streamReader = new StreamReader(stream);

            // 1st line is header (Date,Close/Last,Open,High,Low)
            streamReader.ReadLine();
            var last = new List<double>();
            var open = new List<double>();
            var high = new List<double>();
            var low = new List<double>();
            var date = new List<string>();
            while (!streamReader.EndOfStream)
            {
                var line = streamReader.ReadLine();
                if (string.IsNullOrWhiteSpace(line))
                    continue;

                var split = line.Split(',');
                if (split.Length != 5)
                    continue;

                var values = split.Skip(1).Where(s => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out _)).Select(s => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture)).ToArray();
                last.Add(values[0]);
                open.Add(values[1]);
                high.Add(values[2]);
                low.Add(values[3]);

                date.Add(split[0]);
            }

            last.Reverse();
            open.Reverse();
            high.Reverse();
            low.Reverse();
            date.Reverse();

            this.Series = new ISeries[]
            {
                new LineSeries<double>
                {
                    Values = last,
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0,
                    Name = "Last"
                },
                new LineSeries<double>
                {
                    Values = open,
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0,
                    Name = "Open"
                },
                new LineSeries<double>
                {
                    Values = high,
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0,
                    Name = "High"
                },
                new LineSeries<double>
                {
                    Values = low,
                    Fill = null,
                    GeometrySize = 0,
                    LineSmoothness = 0,
                    Name = "Low"
                }
            };

            this._XAxisMax = date.Count;

            var from = date.First();
            var to = date.Last();
            this.Title = new LabelVisual
            {
                Text = $"NASDAQ Composite Index (COMP) Historical Data [{from} - {to}]",
                TextSize = 16,
                Padding = new LiveChartsCore.Drawing.Padding(15),
                Paint = new SolidColorPaint(SKColors.DarkSlateGray)
            };
            this.XAxes = new[]
            {
                new Axis
                {
                    Name = "Date", Labels = date, LabelsRotation = 90,
                }
            };
            this.YAxes = new[]
            {
                new Axis
                {
                    Name = "Index"
                }
            };
            this.Thumbs = new[]
            {
                new RectangularSection
                {
                    Fill = new SolidColorPaint(new SKColor(255, 205, 210, 100)),
                    Xi = 0,
                    Xj = 0
                }
            };

            this.StartCommand = new DelegateCommand(() =>
            {
                this.ShowThumb(true);

                this._Timer.Interval = TimeSpan.FromSeconds(0.025);
                this._Timer.Tick += this.OnTick;

                this._Timer.Start();

                this.StartCommand?.RaiseCanExecuteChanged();
                this.StopCommand?.RaiseCanExecuteChanged();
            }, () => !this._Timer.IsEnabled);

            this.StopCommand = new DelegateCommand(() =>
            {
                this._Timer.Stop();
                this._Timer.Tick -= this.OnTick;

                this.StartCommand?.RaiseCanExecuteChanged();
                this.StopCommand?.RaiseCanExecuteChanged();
            }, () => this._Timer.IsEnabled);
        }

        #endregion

        #region Methods

        #region Helpers

        private void UpdateThumb(double start, double end)
        {
            var thumb = this.Thumbs[0];
            thumb.Xi = start;
            thumb.Xj = end;
        }

        private void ShowThumb(bool visible)
        {
            var thumb = this.Thumbs[0];

            // IsVisible happen flicking when move thumb from end of x-axis to start of x-ais
            //thumb.IsVisible = visible;
            thumb.Fill = new SolidColorPaint(new SKColor(255, 205, 210, visible ? (byte)100 : (byte)0));
        }

        private void OnTick(object? sender, EventArgs e)
        {
            this._Current++;
            if (this._XAxisMax <= this._Current)
            {
                this._Timer.Stop();
                this._Timer.Tick -= this.OnTick;

                this.StartCommand?.RaiseCanExecuteChanged();
                this.StopCommand?.RaiseCanExecuteChanged();

                this._Current = 0;
                this.ShowThumb(false);
                this.UpdateThumb(0, 0);
                return;
            }

            this.UpdateThumb(this._Current - 0.5, this._Current + 0.5);
        }

        #endregion

        #endregion

        #region IModuleViewModel Members

        public ISeries[] Series
        {
            get;
        }

        public RectangularSection[] Thumbs
        {
            get;
        }

        public LabelVisual Title
        {
            get;
        }

        public Axis[] XAxes
        {
            get;
        }

        public Axis[] YAxes
        {
            get;
        }

        public DelegateCommand StartCommand
        {
            get;
        }

        public DelegateCommand StopCommand
        {
            get;
        }

        #endregion

    }

}
