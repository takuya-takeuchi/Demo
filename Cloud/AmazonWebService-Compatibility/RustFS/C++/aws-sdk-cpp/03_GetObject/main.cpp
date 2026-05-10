#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>

std::shared_ptr<Aws::S3::S3Client> CreateS3Client(const Aws::String& endpoint,
                                                  const Aws::String& region,
                                                  const Aws::String& accessKey = "",
                                                  const Aws::String& secretKey = "")
{
    Aws::S3::S3ClientConfiguration clientConfig;
    clientConfig.region = region;
    clientConfig.endpointOverride = endpoint;
    clientConfig.scheme = Aws::Http::Scheme::HTTP;
    clientConfig.verifySSL = false;

    if (!accessKey.empty() && !secretKey.empty())
    {
        std::cout << "[Info] Use access key and secret key" << std::endl;
        auto credentials = Aws::Auth::AWSCredentials(accessKey, secretKey);
        // Never sign the body of the request
        const auto payloadSigningPolicy = Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::Never;
        const bool useVirtualAddressing = false;
        return std::make_shared<Aws::S3::S3Client>(credentials, clientConfig, payloadSigningPolicy, useVirtualAddressing);
    }
    else
    {
        std::cout << "[Info] Use DefaultAWSCredentialsProviderChain like IAM Role etc" << std::endl;
        return std::make_shared<Aws::S3::S3Client>(clientConfig);
    }
}

bool GetObject(const Aws::String& bucketName, const Aws::String& fileName, std::shared_ptr<Aws::S3::S3Client> s3_client)
{
    Aws::S3::Model::GetObjectRequest request;
    request.SetBucket(bucketName);
    request.SetKey(fileName);

    Aws::S3::Model::GetObjectOutcome outcome = s3_client->GetObject(request);

    if (!outcome.IsSuccess())
    {
        std::cerr << "[Error] GetObject: " << outcome.GetError().GetMessage() << std::endl;
    }
    else
    {
        std::cout << "[Info] Retrieved object '" << fileName << "' from bucket '" << bucketName << "'." << std::endl;
    }

    return outcome.IsSuccess();
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 5)
    {
        std::cerr << "[Error] Demo <endpoint> <bucket_name> <object_name> <region>" << std::endl;
        return -1;
    }

    int32_t error = 0;
    Aws::SDKOptions options;

    try
    {
        const auto endpoint = argv[1];
        const auto bucket_name = argv[2];
        const auto object_name = argv[3];
        const auto region = argv[4];

        std::cout << "[Info]    endpoint: " << endpoint << std::endl;
        std::cout << "[Info] bucket_name: " << bucket_name << std::endl;
        std::cout << "[Info] object_name: " << object_name << std::endl;
        std::cout << "[Info]      region: " << region << std::endl;

        std::cout << "[Info] Aws::InitAPI" << std::endl;
        Aws::InitAPI(options);
        // NOTE
        // accessKey and secretKey shall not be output in console!!
        const char* accessKey = std::getenv("AWS_ACCESS_KEY_ID");
        const char* secretKey = std::getenv("AWS_SECRET_ACCESS_KEY");

        const auto s3Clinet = CreateS3Client(endpoint, region, accessKey ? accessKey : "", secretKey ? secretKey : "");

        GetObject(bucket_name, object_name, s3Clinet);
    }
    catch (const std::runtime_error& re)
    {
        std::cerr << "[Error] " << re.what() << std::endl;
        error = -1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
        error = -1;
    }

    try
    {
        std::cout << "[Info] Aws::ShutdownAPI" << std::endl;
        Aws::ShutdownAPI(options);
        return error;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
}
