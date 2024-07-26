using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;

using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.Painting;
using LiveChartsCore.SkiaSharpView.VisualElements;
using NLog;
using Prism.Mvvm;
using Prism.Regions;
using SkiaSharp;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.ViewModels.Modules
{

    internal sealed class MainModuleViewModel : BindableBase, IModuleViewModel
    {

        #region Fields

        public readonly IRegionManager _RegionManager;

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

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
        }

        #endregion

        #region IModuleViewModel Members

        public ISeries[] Series
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

        #endregion

    }

}
