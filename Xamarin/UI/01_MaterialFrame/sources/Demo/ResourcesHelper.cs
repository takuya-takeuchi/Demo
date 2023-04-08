using System;
using System.Threading.Tasks;

using Xamarin.Forms;

using Sharpnado.MaterialFrame;
using Sharpnado.Tasks;

namespace Demo
{

    public static class ResourcesHelper
    {
        public const string PrimaryColor = nameof(PrimaryColor);

        public const string DynamicMaterialTheme = nameof(DynamicMaterialTheme);
        public const string DynamicBlurStyle = nameof(DynamicBlurStyle);

        public const string DynamicPrimaryColor = nameof(DynamicPrimaryColor);
        public const string DynamicSecondaryColor = nameof(DynamicSecondaryColor);

        public const string DynamicPrimaryOnBackgroundColor = nameof(DynamicPrimaryOnBackgroundColor);
        public const string DynamicSecondaryOnBackgroundColor = nameof(DynamicSecondaryOnBackgroundColor);

        public const string DynamicTextPrimaryColor = nameof(DynamicTextPrimaryColor);
        public const string DynamicTextSecondaryColor = nameof(DynamicTextSecondaryColor);
        public const string DynamicTextTernaryColor = nameof(DynamicTextTernaryColor);

        public const string DynamicHeaderTextColor = nameof(DynamicHeaderTextColor);

        public const string DynamicCornerRadius = nameof(DynamicCornerRadius);

        public const string DynamicIsVisible = nameof(DynamicIsVisible);

        public const string DynamicBackgroundColor = nameof(DynamicBackgroundColor);
        public const string DynamicBackgroundImageSource = nameof(DynamicBackgroundImageSource);

        public const string DynamicLabelAppsColor = nameof(DynamicLabelAppsColor);

        public const string DynamicElevation = nameof(DynamicElevation);

        public static T GetResource<T>(string key)
        {
            if (Application.Current.Resources.TryGetValue(key, out var value))
            {
                return (T)value;
            }

            throw new InvalidOperationException($"key {key} not found in the resource dictionary");
        }

        public static Color GetResourceColor(string key)
        {
            if (Application.Current.Resources.TryGetValue(key, out var value))
            {
                return (Color)value;
            }

            throw new InvalidOperationException($"key {key} not found in the resource dictionary");
        }

        public static void SetDynamicResource(string targetResourceName, string sourceResourceName)
        {
            if (!Application.Current.Resources.TryGetValue(sourceResourceName, out var value))
            {
                throw new InvalidOperationException($"key {sourceResourceName} not found in the resource dictionary");
            }

            Application.Current.Resources[targetResourceName] = value;
        }

        public static void SetDynamicResource<T>(string targetResourceName, T value)
        {
            Application.Current.Resources[targetResourceName] = value;
        }

        public static void SetAcrylic(bool isBlurEnabled)
        {
            SetDynamicResource(DynamicElevation, 0);

            if (isBlurEnabled)
            {
                SetDynamicResource(DynamicMaterialTheme, MaterialFrame.Theme.AcrylicBlur);
                SetDynamicResource(DynamicBackgroundColor, Color.Transparent);
                return;
            }

            SetDynamicResource(DynamicMaterialTheme, MaterialFrame.Theme.Acrylic);
            SetDynamicResource(DynamicBackgroundColor, "AcrylicSurface");
            SetDynamicResource(DynamicBackgroundImageSource, new FileImageSource());
            SetLightColors(false);
        }

        public static void SetBlurStyle(MaterialFrame.BlurStyle blurStyle)
        {
            string wallpaper = string.Empty;
            switch (blurStyle)
            {
                case MaterialFrame.BlurStyle.Light:
                    SetDarkColors(false);
                    wallpaper = "light.png";
                    break;

                case MaterialFrame.BlurStyle.Dark:
                    SetDarkColors(true);
                    wallpaper = "dark.png";
                    break;

                case MaterialFrame.BlurStyle.ExtraLight:
                    SetLightColors(true);
                    wallpaper = "extra_light.png";
                    break;
            }

            TaskMonitor.Create(
                async () =>
                {
                    SetDynamicResource(DynamicBackgroundImageSource, new FileImageSource { File = wallpaper });
                    // give some time to the big picture to load
                    await Task.Delay(100);

                    SetDynamicResource(DynamicBlurStyle, blurStyle);
                });
        }

        public static void SetDarkMode()
        {
            SetDynamicResource(DynamicElevation, 4);
            SetDarkColors(false);
            SetDynamicResource(DynamicBackgroundColor, "DarkSurface");
            SetDynamicResource(DynamicBackgroundImageSource, new FileImageSource());
            SetDynamicResource(DynamicMaterialTheme, MaterialFrame.Theme.Dark);
        }

        public static void SetLightColors(bool darkBackground)
        {
            SetDynamicResource(DynamicHeaderTextColor, "TextPrimaryColor");

            SetDynamicResource(DynamicPrimaryOnBackgroundColor, darkBackground ? "PrimaryDarkColor" : "PrimaryColor");
            SetDynamicResource(DynamicSecondaryOnBackgroundColor, darkBackground ? "SecondaryDarkColor" : "SecondaryColor");

            SetDynamicResource(DynamicPrimaryColor, "PrimaryColor");
            SetDynamicResource(DynamicSecondaryColor, "SecondaryColor");

            SetDynamicResource(DynamicTextPrimaryColor, "TextPrimaryColor");
            SetDynamicResource(DynamicTextSecondaryColor, "TextSecondaryColor");
            SetDynamicResource(DynamicTextTernaryColor, "TextTernaryColor");

            SetDynamicResource(DynamicLabelAppsColor, "LabelAppsColor");
        }

        public static void SetDarkColors(bool lightBackground)
        {
            SetDynamicResource(DynamicHeaderTextColor, "TextPrimaryDarkColor");

            SetDynamicResource(DynamicPrimaryOnBackgroundColor, lightBackground ? "PrimaryColor" : "PrimaryDarkColor");
            SetDynamicResource(DynamicSecondaryOnBackgroundColor, lightBackground ? "SecondaryColor" : "SecondaryDarkColor");

            SetDynamicResource(DynamicPrimaryColor, "PrimaryDarkColor");
            SetDynamicResource(DynamicSecondaryColor, "SecondaryDarkColor");

            SetDynamicResource(DynamicTextPrimaryColor, "TextPrimaryDarkColor");
            SetDynamicResource(DynamicTextSecondaryColor, "TextSecondaryDarkColor");
            SetDynamicResource(DynamicTextTernaryColor, "TextTernaryDarkColor");

            SetDynamicResource(DynamicLabelAppsColor, "LabelAppsDarkColor");
        }
    }

}
