using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace NativeSharp
{

    internal sealed partial class NativeMethods
    {

        [DllImport(NativeLibrary, CallingConvention = CallingConvention)]
        public static extern int native_get_prime_count(int n);

    }

}