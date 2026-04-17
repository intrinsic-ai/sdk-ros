#!/usr/bin/env bash

IMAGES_DIR=./images
BUILDER_NAME=container-builder
ROS_DISTRO=jazzy

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
    --dockerfile)
      CUSTOM_DOCKERFILE="$2"
      shift # past argument
      shift # past value
      ;;
    --dependencies)
      DEPENDENCIES="$2"
      shift # past argument
      shift # past value
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
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

if [[ -n "$SERVICE_NAME" && -n "$SERVICE_PACKAGE" ]]; then
  mkdir -p "$IMAGES_DIR/$SERVICE_NAME"

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.service"
  fi

  docker build -t $SERVICE_PACKAGE:$SERVICE_NAME \
      --file $DOCKERFILE \
      --build-arg="SERVICE_PACKAGE=$SERVICE_PACKAGE" \
      --build-arg="SERVICE_NAME=$SERVICE_NAME" \
      --build-arg="DEPENDENCIES=$DEPENDENCIES" \
      --build-arg="SERVICE_EXECUTABLE_NAME=${SERVICE_NAME}_main" \
      --build-arg="ROS_DISTRO=$ROS_DISTRO" \
      .
  
  echo "INFO: Saving Service image to tarball..."
  docker save "$SERVICE_PACKAGE:$SERVICE_NAME" -o "$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar"

elif [[ -n "$SKILL_NAME" && -n "$SKILL_PACKAGE" ]]; then
  mkdir -p "$IMAGES_DIR/$SKILL_NAME"

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.skill"
  fi

  docker build -t $SKILL_PACKAGE:$SKILL_NAME \
      --file $DOCKERFILE \
      --build-arg="SKILL_PACKAGE=$SKILL_PACKAGE" \
      --build-arg="SKILL_NAME=$SKILL_NAME" \
      --build-arg="SKILL_EXECUTABLE_NAME=${SKILL_NAME}_main" \
      --build-arg="ROS_DISTRO=$ROS_DISTRO" \
      --build-arg="DEPENDENCIES=$DEPENDENCIES" \
      .
  
  echo "INFO: Saving Skill image to tarball..."
  docker save "$SKILL_PACKAGE:$SKILL_NAME" -o "$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.tar"
else
  echo "ERROR: Must provide either service or skill name and package."
  exit 1
fi

echo "INFO: Container build and save complete!"
