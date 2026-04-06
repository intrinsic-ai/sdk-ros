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
    --manifest_path)
      MANIFEST_PATH="$2"
      shift # past argument
      shift # past value
      ;;
    --default_config)
      DEFAULT_CONFIG="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

if [[ -z "$MANIFEST_PATH" ]]; then
  echo "ERROR: --manifest_path must be provided to build_bundle.sh."
  exit 1
fi

if [[ -z "$SERVICE_NAME" && -z "$SKILL_NAME" ]]; then
  echo "ERROR: Either --service_name or --skill_name must be provided."
  exit 1
fi

if [[ -n "$SERVICE_NAME" && -z "$SERVICE_PACKAGE" ]]; then
  echo "ERROR: --service_package must be provided when --service_name is specified."
  exit 1
fi

if [[ -n "$SKILL_NAME" && -z "$SKILL_PACKAGE" ]]; then
  echo "ERROR: --skill_package must be provided when --skill_name is specified."
  exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Parse the SDK_VERSION from sdk_version.json
SDK_VERSION_FILE="$SCRIPT_DIR/../intrinsic_sdk_cmake/cmake/sdk_version.json"
SDK_VERSION=$(grep -oP '"sdk_version": "\K[^"]+' "$SDK_VERSION_FILE")

# Download the 'inbuild' tool if it doesn't exist
if [ ! -f ./inbuild ]; then
    echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
    wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
      && chmod +x inbuild
fi

if [[ -n "$SERVICE_NAME" && -n "$SERVICE_PACKAGE" ]]; then
  echo "INFO: Loading newly built service image into local daemon..."
  docker load -i "$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar"
  
  echo "INFO: Extracting config and descriptor set from container..."
  docker create --name "temp_container_service" "$SERVICE_PACKAGE:$SERVICE_NAME"
  docker cp "temp_container_service:/opt/ros/overlay/install/share/${SERVICE_PACKAGE}/${SERVICE_NAME}_protos.desc" \
   "$IMAGES_DIR/$SERVICE_NAME/${SERVICE_NAME}_protos.desc"
  docker rm -f "temp_container_service"
  
  INBUILD_ARGS=(
    "--file_descriptor_set" "$IMAGES_DIR/$SERVICE_NAME/${SERVICE_NAME}_protos.desc"
    "--manifest" "$MANIFEST_PATH"
    "--oci_image" "$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar"
    "--output" "$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.bundle.tar"
  )

  if [[ -n "$DEFAULT_CONFIG" ]]; then
      INBUILD_ARGS+=("--default_config" "$DEFAULT_CONFIG")
  fi

  echo "INFO: Building the service bundle..."
  ./inbuild service bundle "${INBUILD_ARGS[@]}"

elif [[ -n "$SKILL_NAME" && -n "$SKILL_PACKAGE" ]]; then
  echo "INFO: Loading newly built skill image into local daemon..."
  docker load -i "$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.tar"

  echo "INFO: Extracting descriptor set from container..."
  docker create --name "temp_container_skill" "$SKILL_PACKAGE:$SKILL_NAME"
  docker cp "temp_container_skill:/opt/ros/overlay/install/share/${SKILL_PACKAGE}/${SKILL_NAME}_protos.desc" \
   "$IMAGES_DIR/$SKILL_NAME/${SKILL_NAME}_protos.desc"
  docker rm -f "temp_container_skill"

  echo "INFO: Building the skill bundle..."
  ./inbuild skill bundle \
    --file_descriptor_set "$IMAGES_DIR/$SKILL_NAME/${SKILL_NAME}_protos.desc" \
    --manifest "${MANIFEST_PATH}" \
    --oci_image "$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.tar" \
    --output "$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.bundle.tar"
fi
