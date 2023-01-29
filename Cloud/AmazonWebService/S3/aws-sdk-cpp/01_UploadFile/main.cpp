#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "Test <url> <file path> <content-type>" << std::endl;
        return -1;
    }

    const auto url = argv[1];
    const auto path = argv[2];
    const auto contentType = argv[3];

    Aws::SDKOptions options;
    Aws::InitAPI(options);
    {
        //TODO(user): Change bucket_name to the name of a bucket in your account.
        const Aws::String bucket_name = "<Enter bucket name>";
        //TODO(user): Create a file called "my-file.txt" in the local folder where your executables are built to.
        const Aws::String object_name = "<Enter file>";

        Aws::Client::ClientConfiguration clientConfig;
        // Optional: Set to the AWS Region in which the bucket was created (overrides config file).
        // clientConfig.region = "us-east-1";

        // AwsDoc::S3::PutObject(bucket_name, object_name, clientConfig);
    }

    Aws::ShutdownAPI(options);

    return 0;
}