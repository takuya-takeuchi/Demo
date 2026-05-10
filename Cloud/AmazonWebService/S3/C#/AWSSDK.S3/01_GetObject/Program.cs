using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Amazon.S3;
using Amazon.S3.Model;
using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static async Task Main(string[] args)
        {
            if (args.Length != 3)
            {
                Logger.Error("Usage: Demo <bucket_name> <object_name> <region>");
                return;
            }

            var bucketName = args[0];
            var objectName = args[1];
            var regionName = args[2];

            Logger.Info($"bucket_name: {bucketName}");
            Logger.Info($"object_name: {objectName}");
            Logger.Info($"region: {regionName}");

            Logger.Info("GetObject");
            await GetObject(bucketName, objectName, regionName);
        }

        #region Helpers

        private static async Task<bool> GetObject(string bucketName,
                                                  string fileName,
                                                  string regionName)
        {
            try
            {
                IAmazonS3 client = new AmazonS3Client(Amazon.RegionEndpoint.GetBySystemName(regionName));
                GetObjectRequest request = new GetObjectRequest()
                {
                    BucketName = bucketName,
                    Key = fileName
                };

                var response = await client.GetObjectAsync(request);
                using var reader = new StreamReader(response.ResponseStream);
                var content = reader.ReadToEnd();
                Logger.Info($"Retrieved object '{fileName}' from bucket '{bucketName}'.");
                return true;
            }
            catch (Exception ex)
            {
                Logger.Error($"Error occurred while retrieving S3 object. {ex.Message}");
                return false;
            }
        }

        #endregion

        #endregion

    }

}
