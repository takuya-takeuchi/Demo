using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace NativeSharp
{

    internal sealed partial class NativeMethods
    {

        #region Fields

#if IOS
        public const string NativeLibrary = "__Internal";
#else
        public const string NativeLibrary = "NativeAdd";
#endif

        public const CallingConvention CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl;

        #endregion

    }

}