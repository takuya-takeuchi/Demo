$image_name = "simple-nginx-server"
$container_name = "simple-nginx-server"

docker build -t $image_name .
docker run -d -p 8080:80 `
              --rm `
              -v ${PSScriptRoot}/public:/root/public `
              --name $container_name `
              $image_name