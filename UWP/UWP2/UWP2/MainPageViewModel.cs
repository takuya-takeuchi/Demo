using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;

namespace UWP2
{
    public class MainPageViewModel : INotifyPropertyChanged
    {

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public MainPageViewModel()
        {
            this.Images = this.LoadImages();
        }

        private IEnumerable<ImageSource> _Images;

        public IEnumerable<ImageSource> Images
        {
            get
            {
                return this._Images;
            }
            set
            {
                this._Images = value;
                this.OnPropertyChanged();
            }
        }

        private IEnumerable<ImageSource> LoadImages()
        {
            var paths = new[]
            {
                "ms-appx:///Assets/img100.jpg",
                "ms-appx:///Assets/img101.png",
                "ms-appx:///Assets/img102.jpg",
                "ms-appx:///Assets/img103.png",
                "ms-appx:///Assets/img104.jpg",
                "ms-appx:///Assets/img105.jpg",
            };

            return paths.Select(p => new BitmapImage(new Uri(p)));
        }

    }

}
