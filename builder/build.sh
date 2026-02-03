#!/bin/bash
set -e

SOURCE_IMAGE="https://github.com/CopterExpress/clover/releases/download/v0.25/clover_v0.25.img.zip"
BASE_DIR=$(pwd)
REPO_DIR="${BASE_DIR}/DroneSwarm"
IMAGES_DIR="${BASE_DIR}/images"
TARGET_DIR="/home/pi/DroneSwarm"

mkdir -p ${IMAGES_DIR}
IMAGE_PATH="${IMAGES_DIR}/clover_droneswarm.img"

wget -qO /tmp/clover.zip ${SOURCE_IMAGE}
unzip -p /tmp/clover.zip *.img > ${IMAGE_PATH}

LOOP_DEV=$(sudo kpartx -av ${IMAGE_PATH} | grep -o 'loop[0-9]\+' | head -n1)
MAPPER_BOOT="/dev/mapper/${LOOP_DEV}p1"
MAPPER_ROOT="/dev/mapper/${LOOP_DEV}p2"

MOUNT_POINT="/mnt/clover"
sudo mkdir -p ${MOUNT_POINT}
sudo mount ${MAPPER_ROOT} ${MOUNT_POINT}
sudo mount ${MAPPER_BOOT} ${MOUNT_POINT}/boot

sudo mkdir -p ${MOUNT_POINT}${TARGET_DIR}
sudo cp -r ${REPO_DIR}/* ${MOUNT_POINT}${TARGET_DIR}/
sudo chown -R 1000:1000 ${MOUNT_POINT}${TARGET_DIR}

sudo cp /usr/bin/qemu-arm-static ${MOUNT_POINT}/usr/bin/

sudo chroot ${MOUNT_POINT} /usr/bin/qemu-arm-static /bin/bash -c "
    export LC_ALL=C
    apt-get update
"

sudo umount ${MOUNT_POINT}/boot
sudo umount ${MOUNT_POINT}
sudo kpartx -d ${IMAGE_PATH}