@echo off

set CONTAINER_NAME=simulation-environment
set IMAGE_NAME=simulation-environment-image
set DOCKERFILE=docker/Dockerfile

rem Check if the container is already running
for /f %%i in ('docker ps -a --quiet --filter "status=running" --filter "name=%CONTAINER_NAME%"') do set runningContainer=%%i
if defined runningContainer (
    echo Attaching to running container: %CONTAINER_NAME%
    docker exec -i -t %CONTAINER_NAME% /bin/bash
    exit /b 0
)

echo Building %DOCKERFILE% as image: %IMAGE_NAME%

docker build -f %DOCKERFILE% -t %IMAGE_NAME% .

rem Define Docker arguments
set DOCKER_ARGS=--name %CONTAINER_NAME% -v %cd%/workspace:/home/ubuntu/workspace/ -p 6081:80 --security-opt seccomp=unconfined --shm-size=512m

rem Run the container
docker run -it --rm %DOCKER_ARGS% %IMAGE_NAME%