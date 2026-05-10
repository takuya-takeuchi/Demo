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
            if (args.Length != 4)
            {
                Logger.Error("Usage: Demo <endpoint> <bucket_name> <object_name> <region>");
                return;
            }

            var endpoint = args[0];
            var bucketName = args[1];
            var objectName = args[2];
            var regionName = args[3];

            Logger.Info($"endpoint: {endpoint}");
            Logger.Info($"bucket_name: {bucketName}");
            Logger.Info($"object_name: {objectName}");
            Logger.Info($"region: {regionName}");

            Logger.Info("GetObject");
            await GetObject(endpoint, bucketName, objectName, regionName);
        }

        #region Helpers

        private static async Task<bool> GetObject(string endpoint,
                                                  string bucketName,
                                                  string fileName,
                                                  string regionName)
        {
            try
            {
                IAmazonS3 client;
                
                var uri = new Uri(endpoint);
                var config = new AmazonS3Config
                {
                    ServiceURL = uri.AbsoluteUri,
                    ForcePathStyle = true,
                    UseHttp = uri.AbsoluteUri.StartsWith("http://", StringComparison.OrdinalIgnoreCase)
                };
                
                var regionEndpoint = Amazon.RegionEndpoint.GetBySystemName(regionName);
                var accessKey = Environment.GetEnvironmentVariable("AWS_ACCESS_KEY_ID");
                var secretKey = Environment.GetEnvironmentVariable("AWS_SECRET_ACCESS_KEY");
                if (!string.IsNullOrEmpty(accessKey) && !string.IsNullOrEmpty(secretKey))
                {
                    var credentials = new Amazon.Runtime.BasicAWSCredentials(accessKey, secretKey);
                    client = new AmazonS3Client(credentials, config);
                }
                else
                {
                    // Assume credentials are configured in the environment (e.g., via AWS CLI or EC2 instance profile)
                    client = new AmazonS3Client(config);
                }

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
