using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using GalaSoft.MvvmLight;
using CNTK6.Models;
using Microsoft.MSR.CNTK.Extensibility.Managed;

namespace CNTK6.ViewModels
{

    public class MainViewModel : ViewModelBase
    {

        #region フィールド

        private readonly IDataService _DataService;

        private string _LastHeaderClicked = null;

        private ListSortDirection _LastDirection = ListSortDirection.Ascending;

        #endregion

        #region コンストラクタ

        public MainViewModel(IDataService dataService)
        {
            this._DataService = dataService;
            this._DataService.GetData(
                (item, error) =>
                {
                    if (error != null)
                    {
                        // Report error here
                        return;
                    }
                });
        }

        #endregion

        #region プロパティ

        private string _ConfigFilePath = string.Empty;

        public string ConfigFilePath
        {
            get
            {
                return this._ConfigFilePath;
            }
            set
            {
                this.Set(ref this._ConfigFilePath, value);
            }
        }

        private BitmapImage _Image;

        public BitmapImage Image
        {
            get
            {
                return this._Image;
            }
            set
            {
                this.Set(ref this._Image, value);
            }
        }

        private string _ImagePath = string.Empty;

        public string ImagePath
        {
            get
            {
                return this._ImagePath;
            }
            set
            {
                this.Set(ref this._ImagePath, value);

                Application.Current.Dispatcher.InvokeAsync(() => this.Evaluate(value));
                //Task.Run(() => this.Evaluate(value));
            }
        }

        private string _ModelFilePath = string.Empty;

        public string ModelFilePath
        {
            get
            {
                return this._ModelFilePath;
            }
            set
            {
                this.Set(ref this._ModelFilePath, value);
            }
        }

        private string _LabelMappingFilePath = string.Empty;

        public string LabelMappingFilePath
        {
            get
            {
                return this._LabelMappingFilePath;
            }
            set
            {
                this.Set(ref this._LabelMappingFilePath, value);
            }
        }

        private ICollectionView _Output;

        public ICollectionView Output
        {
            get
            {
                return this._Output;
            }
            set
            {
                this.Set(ref this._Output, value);
            }
        }

        private string _OutputNodeName = string.Empty;

        public string OutputNodeName
        {
            get
            {
                return this._OutputNodeName;
            }
            set
            {
                this.Set(ref this._OutputNodeName, value);
            }
        }

        #endregion

        #region メソッド

        public void Sort(string sortBy)
        {
            if (string.IsNullOrWhiteSpace(sortBy))
            {
                return;
            }

            var viewSource = this._Output;
            var sortDescription = viewSource.SortDescriptions.FirstOrDefault();
            var sortDirection = sortDescription.Direction;

            if (Equals(sortBy, this._LastHeaderClicked))
            {
                sortDirection = sortDirection == ListSortDirection.Descending ?
                    ListSortDirection.Ascending : ListSortDirection.Descending;
            }

            this._LastDirection = sortDirection;
            this._LastHeaderClicked = sortBy;

            using (viewSource.DeferRefresh())
            {
                viewSource.SortDescriptions.Clear();
                var sd = new SortDescription(sortBy, sortDirection);
                viewSource.SortDescriptions.Add(sd);
            }
        }

        #region ヘルパーメソッド

        private void Evaluate(string imagePath)
        {
            this.LoadImage(imagePath);

            var configFilePath = this.ConfigFilePath;
            var modelFilePath = this.ModelFilePath;
            var labelMappingFilePath = this.LabelMappingFilePath;

            // Important
            Environment.CurrentDirectory = Path.GetDirectoryName(this.ConfigFilePath);

            using (var model = new IEvaluateModelManagedF())
            {
                // Initialize model evaluator
                var lines = File.ReadAllLines(configFilePath);
                var config = string.Join("\n", lines);
                model.Init(config);

                // Load model
                model.CreateNetwork(string.Format("deviceId=-1\nmodelPath=\"{0}\"", modelFilePath));

                // Generate random input values in the appropriate structure and size
                var feature = this.GetFeature().ToList();

                var inputs = new Dictionary<string, List<float>>();
                inputs.Add("features", feature);

                var labels = System.IO.File.ReadAllLines(labelMappingFilePath);

                var labelCount = labels.Length;

                var outputNodeName = string.Format("{0}.z", this.OutputNodeName);

                var outputs = model.Evaluate(inputs, outputNodeName, labelCount);

                // convert to probability
                var results = Softmax(outputs);

                var output = labels.Select((label, index) => new CntkResult
                {
                    Label = label,
                    Probability = results[index],
                    ProbabilityText = results[index].ToString("F32")
                }).ToList();

                this.Output = new ListCollectionView(output);
            }
        }

        private void LoadImage(string path)
        {
            var bitmapImage = new BitmapImage();
            bitmapImage.BeginInit();
            bitmapImage.UriSource = new Uri(path);
            bitmapImage.EndInit();
            //bitmapImage.Freeze();

            this.Image = bitmapImage;
        }

        private float[] GetFeature()
        {
            var writeableBitmap = new WriteableBitmap(this.Image);
            var fmt = writeableBitmap.Format;
            var channel = 3;
            if (fmt == PixelFormats.Bgr32)
            {
                channel = 3;
            }
            else if (fmt == PixelFormats.Bgr24)
            {
                channel = 3;
            }
            else if (fmt == PixelFormats.Indexed8)
            {
                channel = 1;
            }
            else if (fmt == PixelFormats.Gray8)
            {
                channel = 1;
            }
            else
            {
                return null;
            }

            var width = writeableBitmap.PixelWidth;
            var height = writeableBitmap.PixelHeight;
            var stride = writeableBitmap.BackBufferStride;
            var res = stride - channel * width;

            writeableBitmap.Lock();

            var rawData = new float[width * height * channel];

            unsafe
            {
                var pBackBuffer = (byte*)(void*)writeableBitmap.BackBuffer;
                if (channel == 1)
                {
                    fixed (float* p = &rawData[0])
                    {
                        var ptr = p;
                        for (var y = 0; y < height; y++)
                        {
                            for (var x = 0; x < width; x++)
                            {
                                ptr[0] = pBackBuffer[0];
                                pBackBuffer += channel;
                                ptr += channel;
                            }

                            pBackBuffer += res;
                        }
                    }
                }
                else
                {
                    fixed (float* p = &rawData[0])
                    {
                        var ptr = p;
                        for (var y = 0; y < height; y++)
                        {
                            for (var x = 0; x < width; x++)
                            {
                                ptr[0] = pBackBuffer[0];
                                ptr[1] = pBackBuffer[1];
                                ptr[2] = pBackBuffer[2];

                                pBackBuffer += channel;
                                ptr += channel;
                            }

                            pBackBuffer += res;
                        }
                    }
                }
            }

            writeableBitmap.Unlock();

            return rawData;
        }

        private IList<double> Softmax(IList<float> outputs)
        {
            var results = new double[outputs.Count];

            var total = 0d;
            for (int index = 0, length = results.Length; index < length; index++)
            {
                results[index] = Math.Exp(outputs[index]);
                total += results[index];
            }

            for (int index = 0, length = results.Length; index < length; index++)
            {
                results[index] /= total;
            }

            return results;
        }

        #endregion

        #endregion

    }

}