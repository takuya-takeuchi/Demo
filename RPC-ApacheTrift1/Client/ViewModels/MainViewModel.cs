using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using ImageProcClient.Contracts;
using ImageProcClient.ViewModels.Interfaces;
using Microsoft.WindowsAPICodePack.Dialogs;
using Thrift.Protocol;
using Thrift.Transport;

namespace ImageProcClient.ViewModels
{

    internal sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region Properties

        private RelayCommand _OpenFileCommand;

        public RelayCommand OpenFileCommand
        {
            get
            {
                return this._OpenFileCommand ?? (this._OpenFileCommand = new RelayCommand(() =>
                {
                    using (var dlg = new CommonOpenFileDialog())
                    {
                        dlg.IsFolderPicker = false;
                        dlg.AddToMostRecentlyUsedList = false;
                        dlg.AllowNonFileSystemItems = false;
                        dlg.EnsureFileExists = true;
                        dlg.EnsurePathExists = true;
                        dlg.EnsureReadOnly = false;
                        dlg.EnsureValidNames = true;
                        dlg.Multiselect = false;
                        dlg.ShowPlacesList = true;

                        var dialogResult = dlg.ShowDialog();
                        if (dialogResult != CommonFileDialogResult.Ok)
                            return;

                        var bitmap = new BitmapImage();
                        try
                        {
                            bitmap.BeginInit();
                            bitmap.UriSource = new Uri(dlg.FileName);
                            bitmap.EndInit();
                            this.SourceImage = bitmap;
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show(ex.Message);
                        }
                    }

                    this._ServerRequestCommand?.RaiseCanExecuteChanged();
                }, () => true));
            }
        }

        private ImageSource _ResultImage;

        public ImageSource ResultImage
        {
            get
            {
                return this._ResultImage;
            }
            private set
            {
                this._ResultImage = value;
                this.RaisePropertyChanged();
            }
        }

        private RelayCommand _ServerRequestCommand;

        public RelayCommand ServerRequestCommand
        {
            get
            {
                return this._ServerRequestCommand ?? (this._ServerRequestCommand = new RelayCommand(() =>
                {
                    var bitmap = this._SourceImage as BitmapImage;
                    if (bitmap == null)
                        return;

                    var width = bitmap.PixelWidth;
                    var height = bitmap.PixelHeight;
                    var stride = (width * bitmap.Format.BitsPerPixel + 7) / 8;
                    var bitsPerPixel = bitmap.Format.BitsPerPixel;
                    var bytesPerPixel = bitsPerPixel / 8;
                    var originalPixels = new byte[width * height * bytesPerPixel];
                    bitmap.CopyPixels(originalPixels, stride, 0);


                    try
                    {
                        TTransport transport = new TSocket("localhost", 9090);
                        TProtocol protocol = new TBinaryProtocol(transport);
                        imageProcService.Client client = new imageProcService.Client(protocol);

                        transport.Open();
                        try
                        {
                            client.send_Revert(originalPixels, width, height);
                            var resultPixels = client.recv_Revert();
                            if (resultPixels != null)
                            {
                                var bmpSource = BitmapSource.Create(
                                    width,
                                    height,
                                    bitmap.DpiX,
                                    bitmap.DpiY,
                                    bitmap.Format,
                                    null,
                                    resultPixels,
                                    stride);
                                if (bmpSource.CanFreeze)
                                    bmpSource.Freeze();

                                this.ResultImage = bmpSource;
                            }
                        }
                        finally
                        {
                            transport.Close();
                        }
                    }
                    catch (Exception x)
                    {
                        Console.WriteLine(x.StackTrace);
                    }

                }, () => this._SourceImage != null));
            }
        }

        private ImageSource _SourceImage;

        public ImageSource SourceImage
        {
            get
            {
                return this._SourceImage;
            }
            private set
            {
                this._SourceImage = value;
                this.RaisePropertyChanged();
            }
        }

        #endregion

    }

}