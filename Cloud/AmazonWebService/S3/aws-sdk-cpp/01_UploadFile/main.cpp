#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>

// https://github.com/awsdocs/aws-doc-sdk-examples/blob/main/cpp/example_code/s3/put_object.cpp
bool PutObject(const Aws::String &bucketName,
               const Aws::String &fileName,
               const Aws::Client::ClientConfiguration &clientConfig) {
    Aws::S3::S3Client s3_client(clientConfig);

    Aws::S3::Model::PutObjectRequest request;
    request.SetBucket(bucketName);
    //We are using the name of the file as the key for the object in the bucket.
    //However, this is just a string and can be set according to your retrieval needs.
    request.SetKey(fileName);

    std::shared_ptr<Aws::IOStream> inputData =
            Aws::MakeShared<Aws::FStream>("SampleAllocationTag",
                                          fileName.c_str(),
                                          std::ios_base::in | std::ios_base::binary);

    if (!*inputData) {
        std::cerr << "Error unable to read file " << fileName << std::endl;
        return false;
    }

    request.SetBody(inputData);

    Aws::S3::Model::PutObjectOutcome outcome =
            s3_client.PutObject(request);

    if (!outcome.IsSuccess()) {
        std::cerr << "Error: PutObject: " <<
                  outcome.GetError().GetMessage() << std::endl;
    }
    else {
        std::cout << "Added object '" << fileName << "' to bucket '"
                  << bucketName << "'.";
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

        std::cout << "[Info] PutObject" << std::endl;
        PutObject(bucket_name, object_name, clientConfig);
        std::cout << "" << std::endl;
    }

    std::cout << "[Info] Aws::ShutdownAPI" << std::endl;
    Aws::ShutdownAPI(options);

    return 0;
}