using System;
using System.Runtime.InteropServices;

// https://github.com/microsoft/microsoft-ui-xaml/issues/2828#issuecomment-653825561
namespace _01_Win32API
{

    [ComImport]
    [InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    [Guid("EECDBF0E-BAE9-4CB6-A68E-9598E1CB57BB")]
    public interface IWindowNative
    {

        IntPtr WindowHandle
        {
            get;
        }

    }

}
