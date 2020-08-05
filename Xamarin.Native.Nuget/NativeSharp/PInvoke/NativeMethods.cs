using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace NativeSharp
{

    internal sealed partial class NativeMethods
    {

        #region Fields

#if XAMARINIOS
        public const string NativeLibrary = "__Internal";
#else
        public const string NativeLibrary = "DlibDotNetNative";
#endif

        public const CallingConvention CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl;

        #endregion

    }

}