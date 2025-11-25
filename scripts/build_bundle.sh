#!/usr/bin/env bash

IMAGES_DIR=./images
BUILDER_NAME=container-builder

while [[ $# -gt 0 ]]; do
  case $1 in
    --images_dir)
      IMAGES_DIR="$2"
      shift # past argument
      shift # past value
      ;;
    --builder_name)
      BUILDER_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --service_name)
      SERVICE_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --service_package)
      SERVICE_PACKAGE="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

podman load --input images/$SERVICE_NAME/$SERVICE_NAME.tar
podman create "$SERVICE_PACKAGE:$SERVICE_NAME" > images/$SERVICE_NAME/container_id.txt
CONTAINER_ID=$(cat images/$SERVICE_NAME/container_id.txt)

podman cp "$CONTAINER_ID:/service_manifest.binarypb" images/$SERVICE_NAME/service_manifest.binarypb

podman start "$CONTAINER_ID"

if podman exec "$CONTAINER_ID" test -f /default_config.binarypb; then
  podman cp "$CONTAINER_ID:/default_config.binarypb" images/$SERVICE_NAME/default_config.binarypb
fi

if podman exec "$CONTAINER_ID" test -f /parameter-descriptor-set.proto.bin; then
  podman cp "$CONTAINER_ID:/parameter-descriptor-set.proto.bin" images/$SERVICE_NAME/parameter-descriptor-set.proto.bin
fi

podman stop "$CONTAINER_ID"
podman rm $CONTAINER_ID
chmod 644 images/$SERVICE_NAME/$SERVICE_NAME.tar

TAR_FILES="$SERVICE_NAME.tar service_manifest.binarypb"
if [ -f default_config.binarypb ]; then
  TAR_FILES="$TAR_FILES default_config.binarypb"
fi
if [ -f parameter-descriptor-set.proto.bin ]; then
  TAR_FILES="$TAR_FILES parameter-descriptor-set.proto.bin"
fi

tar -cvf images/$SERVICE_NAME.bundle.tar \
  --owner=0 \
  --group=0 \
  --no-same-owner \
  --no-same-permissions \
  -C \
  images/$SERVICE_NAME/ \
  $TAR_FILES
