using Xamarin.Forms;

namespace Xamarin.Camera.Controls
{

    public sealed class CameraPreview : View
    {

        #region Properties

        public static readonly BindableProperty CameraProperty = BindableProperty.Create(
            propertyName: "Camera",
            returnType: typeof(CameraOptions),
            declaringType: typeof(CameraPreview),
            defaultValue: CameraOptions.Rear);

        public CameraOptions Camera
        {
            get => (CameraOptions)GetValue(CameraProperty);
            set => this.SetValue(CameraProperty, value);
        }

        public static readonly BindableProperty IsPreviewingProperty = BindableProperty.Create(
            propertyName: "IsPreviewing",
            returnType: typeof(bool),
            declaringType: typeof(CameraPreview),
            defaultValue: false);

        public bool IsPreviewing
        {
            get => (bool)GetValue(IsPreviewingProperty);
            set => this.SetValue(IsPreviewingProperty, value);
        }

        //とりあえず何かやりとりするプロパティ
        public static readonly BindableProperty HogeProperty = BindableProperty.Create(
            propertyName: "Hoge",
            returnType: typeof(object),
            declaringType: typeof(CameraPreview),
            defaultValue: null);

        public object Hoge
        {
            get => (object)GetValue(HogeProperty);
            set => this.SetValue(HogeProperty, value);
        }

        #endregion

    }

}
