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

podman load --input $IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar
podman create "$SERVICE_PACKAGE:$SERVICE_NAME" > $IMAGES_DIR/$SERVICE_NAME/container_id.txt
CONTAINER_ID=$(cat $IMAGES_DIR/$SERVICE_NAME/container_id.txt)

podman cp "$CONTAINER_ID:/service_manifest.binarypb" $IMAGES_DIR/$SERVICE_NAME/service_manifest.binarypb


for FILE in default_config.binarypb parameter-descriptor-set.proto.bin
do
  podman cp "$CONTAINER_ID:/$FILE" $IMAGES_DIR/$SERVICE_NAME/$FILE 2>/dev/null || true
done

podman rm $CONTAINER_ID
chmod 644 $IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar

TAR_FILES="$SERVICE_NAME.tar service_manifest.binarypb"
for FILE in default_config.binarypb parameter-descriptor-set.proto.bin
do
  if [ -f "$IMAGES_DIR/$SERVICE_NAME/$FILE" ]; then
    TAR_FILES="$TAR_FILES $FILE"
  fi
done

tar -cvf $IMAGES_DIR/$SERVICE_NAME.bundle.tar \
  --owner=0 \
  --group=0 \
  --no-same-owner \
  --no-same-permissions \
  -C \
  $IMAGES_DIR/$SERVICE_NAME/ \
  $TAR_FILES
