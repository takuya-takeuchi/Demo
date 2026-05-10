import argparse
import os

import boto3
from botocore.client import Config

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--endpoint", type=str, required=True)
    parser.add_argument("-b", "--bucket-name", type=str, required=True)
    parser.add_argument("-k", "--key", type=str, required=True)
    parser.add_argument("-r", "--region", type=str, required=True)
    return parser.parse_args()

def run(endpoint: str, bucket_name: str, key: str, region: str):
    try:        
        aws_access_key_id = os.getenv("AWS_ACCESS_KEY_ID");
        aws_secret_access_key = os.getenv("AWS_SECRET_ACCESS_KEY");
        if aws_access_key_id and aws_secret_access_key:
            client = boto3.client('s3',
                                  aws_access_key_id=aws_access_key_id,
                                  aws_secret_access_key=aws_secret_access_key,
                                  endpoint_url=endpoint,
                                  region_name=region,
                                  config=Config(
                                    s3={
                                        'addressing_style': 'virtual'
                                    }
                                  )
            )
        else:
            # Assume credentials are configured in the environment (e.g., via AWS CLI or EC2 instance profile)
            client = boto3.client('s3',
                                  endpoint_url=endpoint,
                                  region_name=region,
                                  config=Config(
                                    s3={
                                        'addressing_style': 'virtual'
                                    }
                                  )
            )
        
        response = client.get_object(Bucket=bucket_name, Key=key)
        content = response['Body'].read()
        print(f"[Info] Retrieved object '{key}' from bucket '{bucket_name}'.")
    except Exception as e:
        print(f"[Error] Error occurred: {e}")

if __name__ == '__main__':
    # parse args
    args = get_args()
    endpoint    = args.endpoint
    bucket_name = args.bucket_name
    key         = args.key
    region      = args.region

    print("Arguments")
    print("     endpoint: {}".format(endpoint))
    print("  bucket_name: {}".format(bucket_name))
    print("          key: {}".format(key))
    print("       region: {}".format(region))

    run(endpoint, bucket_name, key, region)