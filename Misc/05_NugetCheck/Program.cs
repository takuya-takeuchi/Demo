using System;
using System.IO;
using System.Linq;
using NuGet.Common;
using NuGet.Packaging;

namespace NugetCheck
{

    internal class Program
    {

        #region Methods

        private static void Main(string[] args)
        {
            using var fs = new FileStream(args[0], FileMode.Open, FileAccess.Read, FileShare.Read);
            using var nuGetPackage = new PackageArchiveReader(fs, leaveStreamOpen: false);
            var duplicate = HasDuplicatedEntries(nuGetPackage);
            if (!duplicate)
            {
                Console.WriteLine("No problem");
                return;
            }

            Console.WriteLine("The package contains one or more duplicated files in the same folder.");
            var packageFiles = nuGetPackage.GetFiles().Select(FileNameHelper.GetZipEntryPath).ToList();
            packageFiles.Sort();
            foreach (var packageFile in packageFiles)
                Console.WriteLine($"\t{packageFile}");

            var count = packageFiles.Count();
            var count2 = packageFiles.Distinct(StringComparer.OrdinalIgnoreCase).Count();
            Console.WriteLine();
            Console.WriteLine($"count: {count}, distinct: {count2}");
        }

        #region Helpers

        private static bool HasDuplicatedEntries(PackageArchiveReader nuGetPackage)
        {
            // Normalize paths and ensures case sensitivity is also considered
            var packageFiles = nuGetPackage.GetFiles().Select(FileNameHelper.GetZipEntryPath);

            return packageFiles.Count() != packageFiles.Distinct(StringComparer.OrdinalIgnoreCase).Count();
        }

        #endregion

        #endregion

        public static class FileNameHelper
        {
            
            public static string GetZipEntryPath(string filePath)
            {
                if (filePath == null)
                {
                    throw new ArgumentNullException(nameof(filePath));
                }

                return PathUtility.StripLeadingDirectorySeparators(filePath);
            }
        }

    }

}
