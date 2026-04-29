#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>

std::shared_ptr<Aws::S3::S3Client> CreateS3Client(const Aws::String &endpoint,
                                                  const Aws::String &region,
                                                  const Aws::String &accessKey = "",
                                                  const Aws::String &secretKey = "")
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

void UploadFile(const Aws::String &bucket,
                const Aws::String &key,
                const Aws::String &filePath,
                std::shared_ptr<Aws::S3::S3Client> s3_client)
{
    Aws::S3::Model::PutObjectRequest request;
    request.SetBucket(bucket);
    request.SetKey(key);

    const auto input_data =
        Aws::MakeShared<Aws::FStream>("PutObjectStream", filePath.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!*input_data)
        throw std::runtime_error("Failed to open file: " + filePath);

    request.SetBody(input_data);

    auto outcome = s3_client->PutObject(request);
    if (!outcome.IsSuccess())
        throw std::runtime_error(outcome.GetError().GetMessage());
}

int32_t main(int32_t argc, const char **argv)
{
    if (argc != 3)
    {
        std::cerr << "[Error] Demo <endpoint> <region>" << std::endl;
        return -1;
    }

    int32_t error = 0;
    Aws::SDKOptions options;

    try
    {

        const auto endpoint = argv[1];
        const auto region = argv[2];

        std::cout << "[Info]    endpoint: " << endpoint << std::endl;
        std::cout << "[Info]      region: " << region << std::endl;

        std::cout << "[Info] Aws::InitAPI" << std::endl;
        Aws::InitAPI(options);
        // NOTE
        // accessKey and secretKey shall not be output in console!!
        const char *accessKey = std::getenv("AWS_ACCESS_KEY_ID");
        const char *secretKey = std::getenv("AWS_SECRET_ACCESS_KEY");

        const auto s3Clinet = CreateS3Client(endpoint, region, accessKey ? accessKey : "", secretKey ? secretKey : "");

        // List buckets
        auto response = s3Clinet->ListBuckets();
        if (response.IsSuccess())
        {
            // Parse response
            auto buckets = response.GetResult().GetBuckets();
            for (auto iter = buckets.begin(); iter != buckets.end(); ++iter)
            {
                // cout bucket name and creation date.
                std::cout << iter->GetName() << "\t"
                          << iter->GetCreationDate().ToLocalTimeString(Aws::Utils::DateFormat::ISO_8601) << std::endl;
            }
        }
        else
        {
            // Error handle.
            std::cerr << "[Error] while ListBuckets " << response.GetError().GetExceptionName() << " "
                      << response.GetError().GetMessage() << std::endl;
        }
    }
    catch (const std::runtime_error &re)
    {
        std::cerr << "[Error] " << re.what() << std::endl;
        error = -1;
    }
    catch (const std::exception &e)
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
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
}
