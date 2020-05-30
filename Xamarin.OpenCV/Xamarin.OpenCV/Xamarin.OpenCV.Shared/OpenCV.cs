using System;
using System.Runtime.InteropServices;

namespace Xamarin.OpenCV
{

    internal static class OpenCV
    {

        #region Methods

        public static string GetBuildInformation()
        {
            var str = NativeMethods.opencv_getBuildInformation();
            var cstr = NativeMethods.string_c_str(str);
            var ret = Marshal.PtrToStringAnsi(cstr);
            if (str != IntPtr.Zero)
                NativeMethods.string_delete(str);

            return ret;
        }

        public static string GetVersion()
        {
            var str = NativeMethods.opencv_get_version();
            var cstr = NativeMethods.string_c_str(str);
            var ret = Marshal.PtrToStringAnsi(cstr);
            if (str != IntPtr.Zero)
                NativeMethods.string_delete(str);

            return ret;
        }

        #endregion

    }

}
