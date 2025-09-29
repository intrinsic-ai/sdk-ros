#!/usr/bin/env bash
set -o errexit
set -x

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
INBUILD=$(pwd)/install/intrinsic_sdk_cmake/bin/inbuild

IMAGE_DIR=$(pwd)/images/$SKILL_NAME
mkdir -p ${IMAGE_DIR}

cd "$PACKAGE_DIR"

echo "Starting podman build..."
podman build -f "$DOCKERFILE" \
  --build-arg SKILL_NAME="$SKILL_NAME" \
  --build-arg SKILL_PACKAGE="$SKILL_PACKAGE" \
  --build-arg SKILL_EXECUTABLE="lib/${SKILL_PACKAGE}/${SKILL_NAME}_main" \
  --build-arg SKILL_CONFIG="share/${SKILL_PACKAGE}/${SKILL_NAME}_config.pbbin" \
  --build-arg SKILL_ASSET_ID_ORG="$SKILL_ASSET_ID_ORG" \
  --build-arg SKILL_MANIFEST_TEXTPROTO="share/${SKILL_PACKAGE}/${SKILL_NAME}.manifest.textproto" \
  --build-arg SKILL_MANIFEST_FILE_DESCRIPTOR_SET="share/${SKILL_PACKAGE}/${SKILL_NAME}_protos.desc" \
  --tag "${SKILL_NAME}:${CONTAINER_TAG_NAME}" \
  .

echo "Starting podman save..."
podman save \
  --format="oci-archive" \
  --output="${IMAGE_DIR}/${SKILL_NAME}.tar" \
  "${SKILL_NAME}:${CONTAINER_TAG_NAME}"

echo "Instantiating image..."
podman create "${SKILL_NAME}:${CONTAINER_TAG_NAME}" > ${IMAGE_DIR}/container_id.txt
CONTAINER_ID=$(cat ${IMAGE_DIR}/container_id.txt)

echo "Extracting manifest and file descriptor set..."
podman cp "${CONTAINER_ID}:/skills/skill_manifest.textproto" ${IMAGE_DIR}/skill_manifest.textproto
podman cp "${CONTAINER_ID}:/skills/skill_protos.desc" ${IMAGE_DIR}/skill_protos.desc
podman rm ${CONTAINER_ID}

${INBUILD} skill bundle \
 --output ${IMAGE_DIR}/../${SKILL_NAME}.bundle.tar \
 --manifest ${IMAGE_DIR}/skill_manifest.textproto \
 --file_descriptor_set ${IMAGE_DIR}/skill_protos.desc \
 --oci_image ${IMAGE_DIR}/${SKILL_NAME}.tar
