#!/bin/bash

IMAGE_NAME=$1
if [[ -z $IMAGE_NAME ]]; then
	echo "Usage: $0 <image name>"
	exit 1
fi

docker buildx create \
	--name feldfreund_devkit_ros \
	--use \
	--bootstrap

docker buildx build \
	--platform linux/amd64,linux/arm64 \
	-t "${IMAGE_NAME}" \
	--push \
	-f Dockerfile \
	..
