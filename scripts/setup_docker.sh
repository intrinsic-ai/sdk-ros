#!/usr/bin/env bash

echo "=================================================================================================" >&2
echo "WARNING: This script is deprecated and will be removed in a future release." >&2
echo "Please transition to using the new Python library: intrinsic_sdk_bundle_library_py" >&2
echo "=================================================================================================" >&2

IMAGES_DIR=images
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
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

mkdir -p $IMAGES_DIR
docker buildx inspect --builder $BUILDER_NAME || \
  docker buildx create --name="$BUILDER_NAME" --driver="docker-container"

echo "images
build
log" > ./.dockerignore
