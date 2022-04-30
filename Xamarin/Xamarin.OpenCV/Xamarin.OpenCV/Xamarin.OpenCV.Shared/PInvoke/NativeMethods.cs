using System;
using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace Xamarin.OpenCV
{

    internal sealed partial class NativeMethods
    {

        #region Fields

#if Android
    const string NativeLibrary = "libOpenCVWrapper.so";
#else
        const string NativeLibrary = "__Internal";
#endif

        [DllImport(NativeLibrary)]
        public static extern IntPtr opencv_get_version();

        [DllImport(NativeLibrary)]
        public static extern IntPtr opencv_getBuildInformation();

        [DllImport(NativeLibrary)]
        public static extern IntPtr string_c_str(IntPtr str);

        [DllImport(NativeLibrary)]
        public static extern void string_delete(IntPtr str);

        #endregion

    }

}
