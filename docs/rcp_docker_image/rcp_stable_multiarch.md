###### rcp image build and push (multi-architecture image)

- A useful docker multi-arch images [guide](https://medium.com/@artur.klauser/building-multi-architecture-docker-images-with-buildx-27d80f7e2408)

- This image will be compatible with two platforms:
	- linux/amd64
	- linux/arm/v7

```console
user@host:~$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
user@host:~$ docker buildx create --name mybuilder
mybuilder
user@host:~$ docker buildx use mybuilder
user@host:~$ docker buildx inspect --bootstrap
user@host:~$ export DOCKER_USER="your_docker_username"
user@host:~$ docker login -u "$DOCKER_USER"
user@host:~$ docker buildx build -t isrlab/rcp:stable --platform linux/amd64,linux/arm/v7 --push .
```