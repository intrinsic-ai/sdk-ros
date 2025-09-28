#!/usr/bin/env bash

if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace."
  exit
fi

SKILL_ASSET_ID_ORG=com.example
CONTAINER_TAG_NAME=latest

while [[ $# -gt 0 ]]; do
  case $1 in
    --skill_name)
      SKILL_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --skill_package)
      SKILL_PACKAGE="$2"
      shift # past argument
      shift # past value
      ;;
    --skill_asset_id_org)
      SKILL_ASSET_ID_ORG="$2"
      shift # past argument
      shift # past value
      ;;
    --tag)
      CONTAINER_TAG_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

if [[ -z "$SKILL_NAME" || -z "$SKILL_PACKAGE" ]]; then
  echo "Must provide --skill_package and --skill_name."
  exit 1
fi

if [[ $(find src -type d -name "$SKILL_PACKAGE" | wc -l) -eq "0" ]]; then
  echo "A directory named $SKILL_PACKAGE was not found."
  exit 1
elif [[ $(find src -type d -name "$SKILL_PACKAGE" | wc -l) -ne "1" ]]; then
  echo "Multiple directories named $SKILL_PACKAGE were found. Cannot proceed."
  exit 1
fi

PACKAGE_DIR=$(find src -type d -name "$SKILL_PACKAGE")
echo $PACKAGE_DIR
if [[ ! -f "${PACKAGE_DIR}/package.xml" ]]; then
  echo "Package directory $PACKAGE_DIR does not have a package.xml file."
  exit 1
fi

DOCKERFILE=$(pwd)/src/sdk-ros/intrinsic_sdk_cmake/cmake/api/skill/resource/skill.Dockerfile

mkdir -p "images/$SKILL_NAME"
IMAGE_DIR=$(pwd)/images/$SKILL_NAME

cd "$PACKAGE_DIR"

echo "Starting podman build..."
podman build -f "$DOCKERFILE" \
  --build-arg SKILL_NAME="$SKILL_NAME" \
  --build-arg SKILL_PACKAGE="$SKILL_PACKAGE" \
  --build-arg SKILL_EXECUTABLE="lib/${SKILL_PACKAGE}/${SKILL_NAME}_main" \
  --build-arg SKILL_CONFIG="share/${SKILL_PACKAGE}/${SKILL_NAME}_config.pbbin" \
  --build-arg SKILL_ASSET_ID_ORG="$SKILL_ASSET_ID_ORG" \
  --tag "$CONTAINER_TAG_NAME" \
  .

echo "Starting podman save..."
podman save \
  --format="docker-archive" \
  --output="${IMAGE_DIR}/${SKILL_NAME}.tar" \
  "$CONTAINER_TAG_NAME"
