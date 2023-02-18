#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 6)
    {
        std::cout << "[Error] Test <bucket_name> <object_name> <region> <method> <expirationInSeconds>" << std::endl;
        return -1;
    }

    std::cout << "        bucket_name: " << argv[1] << std::endl;
    std::cout << "        object_name: " << argv[2] << std::endl;
    std::cout << "             region: " << argv[3] << std::endl;
    std::cout << "             method: " << argv[4] << std::endl;
    std::cout << "expirationInSeconds: " << argv[5] << std::endl;

    Aws::Http::HttpMethod method;
    const auto tmp = std::string(argv[4]);
    if      (tmp == "GET") method = Aws::Http::HttpMethod::HTTP_GET;
    else if (tmp == "PUT") method = Aws::Http::HttpMethod::HTTP_PUT;
    else
    {
        std::cout << "[Error] '" << tmp << "' is not supported. Supported methods are only GET or PUT" << std::endl;
        return -2;
    }

    Aws::SDKOptions options;

    std::cout << "[Info] Aws::InitAPI" << std::endl;
    Aws::InitAPI(options);
    {
        const Aws::String bucket_name = Aws::String(argv[1]);
        const Aws::String object_name = Aws::String(argv[2]);
        const Aws::String region_name = Aws::String(argv[3]);
        const Aws::String method_name = Aws::String(argv[4]);
        const uint64_t expirationInSeconds = atoll(argv[5]);

        Aws::Client::ClientConfiguration clientConfig;
        clientConfig.region = region_name;

        std::cout << "[Info] Generate Presigned Url" << std::endl;

        Aws::S3::S3Client client(clientConfig);
        const auto presignedUrl = client.GeneratePresignedUrl(bucket_name,
                                                              object_name,
                                                              method,
                                                              expirationInSeconds);

        std::cout << presignedUrl.c_str() << std::endl;
    }

    std::cout << "[Info] Aws::ShutdownAPI" << std::endl;
    Aws::ShutdownAPI(options);

    return 0;
}