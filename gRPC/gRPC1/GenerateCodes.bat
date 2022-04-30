python -m grpc_tools.protoc -I. --python_out=python --grpc_python_out=python imageProc.proto
mkdir gRPC\Contracts
packages\Grpc.Tools.1.2.0\tools\windows_x86\protoc.exe -I. --csharp_out gRPC\Contracts --grpc_out gRPC\Contracts imageProc.proto --plugin=protoc-gen-grpc=packages\Grpc.Tools.1.2.0\tools\windows_x86\grpc_csharp_plugin.exe