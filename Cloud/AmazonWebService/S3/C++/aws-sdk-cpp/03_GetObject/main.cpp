#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>

bool GetObject(const Aws::String& bucketName,
               const Aws::String& fileName,
               const Aws::Client::ClientConfiguration& clientConfig)
{
    Aws::S3::S3Client s3_client(clientConfig);

    Aws::S3::Model::GetObjectRequest request;
    request.SetBucket(bucketName);
    request.SetKey(fileName);

    Aws::S3::Model::GetObjectOutcome outcome = s3_client.GetObject(request);

    if (!outcome.IsSuccess())
    {
        std::cerr << "[Error] GetObject: " << outcome.GetError().GetMessage() << std::endl;
    }
    else
    {
        auto& retrieved_file = outcome.GetResultWithOwnership().GetBody();
        std::cout << "[Info] Retrieved object '" << fileName << "' from bucket '" << bucketName << "' size: " << retrieved_file.tellp() << " bytes" << std::endl;
    }

    return outcome.IsSuccess();
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "Test <bucket_name> <object_name> <region>" << std::endl;
        return -1;
    }

    std::cout << "bucket_name: " << argv[1] << std::endl;
    std::cout << "object_name: " << argv[2] << std::endl;
    std::cout << "     region: " << argv[3] << std::endl;

    Aws::SDKOptions options;

    std::cout << "[Info] Aws::InitAPI" << std::endl;
    Aws::InitAPI(options);
    {
        const Aws::String bucket_name = Aws::String(argv[1]);
        const Aws::String object_name = Aws::String(argv[2]);
        const Aws::String region_name = Aws::String(argv[3]);

        Aws::Client::ClientConfiguration clientConfig;
        clientConfig.region = region_name;

        std::cout << "[Info] GetObject" << std::endl;
        GetObject(bucket_name, object_name, clientConfig);
    }

    std::cout << "[Info] Aws::ShutdownAPI" << std::endl;
    Aws::ShutdownAPI(options);

    return 0;
}
