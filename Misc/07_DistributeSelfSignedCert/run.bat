docker build -t nginx-cert .
docker run --name nginx-cert ^
           -d -v %cd%\src:/usr/share/nginx/html ^
           -p 80:80 nginx-cert:latest