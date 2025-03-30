using System;
using System.Diagnostics;
using System.Linq;

using BenchmarkDotNet.Running;

namespace Demo
{

    internal sealed class Program
    {

        #region Methods

        private static void Main(string[] args)
        {
            var summary = BenchmarkRunner.Run<Benchmark>();
        }

        #endregion

    }

}