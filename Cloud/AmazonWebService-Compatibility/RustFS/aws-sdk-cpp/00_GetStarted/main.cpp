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

    if (!accessKey.empty() && !secretKey.empty())
    {
        // 1. キーが渡された場合：StaticCredentialsProviderを使用
        auto credentials = Aws::Auth::AWSCredentials(accessKey, secretKey);
        // Never sign the body of the request
        const auto payloadSigningPolicy = Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::Never;
        const bool useVirtualAddressing = false;
        return std::make_shared<Aws::S3::S3Client>(credentials, clientConfig, payloadSigningPolicy, useVirtualAddressing);
    }
    else
    {
        // 2. キーが空の場合：DefaultAWSCredentialsProviderChainを使用 (IAM Role等を自動取得)
        // コンストラクタにproviderを渡さないことでデフォルト挙動になる
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
    if (argc != 6)
    {
        std::cerr << "[Error] Demo <endpoint> <bucket_name> <object_name> <region> <filepath>" << std::endl;
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
        const auto filepath = argv[5];

        std::cout << "[Info]    endpoint: " << endpoint << std::endl;
        std::cout << "[Info] bucket_name: " << bucket_name << std::endl;
        std::cout << "[Info] object_name: " << object_name << std::endl;
        std::cout << "[Info]      region: " << region << std::endl;
        std::cout << "[Info]    filepath: " << filepath << std::endl;

        std::cout << "[Info] Aws::InitAPI" << std::endl;
        Aws::InitAPI(options);

        // NOTE
        // accessKey and secretKey shall not be output in console!!
        const char *accessKey = std::getenv("AWS_ACCESS_KEY");
        const char *secretKey = std::getenv("AWS_SECRET_ACCESS_KEY");

        const auto s3Clinet = CreateS3Client(endpoint, region, accessKey, secretKey);

        UploadFile(bucket_name, "", filepath, s3Clinet);
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
