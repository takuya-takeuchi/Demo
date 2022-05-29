using System;

namespace NativeSharp
{
    public static class Native
    {

        public static int GetPrimeCount(int n)
        {
            return NativeSharp.NativeMethods.native_get_prime_count(n);
        }

    }

}