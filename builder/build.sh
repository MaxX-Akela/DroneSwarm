#! /usr/bin/env bash
set -e 

SOURCE_IMAGE="https://github.com/CopterExpress/clover/releases/download/v0.25/clover_v0.25.img.zip"
REPO_DIR="/mnt/DroneSwarm"
IMAGES_DIR="/mnt/images"
SCRIPTS_DIR="${REPO_DIR}/builder"
TARGET_DIR="/home/pi/DroneSwarm"

echo_stamp() {
  echo -e "\e[1m\e[34m$(date '+[%Y-%m-%d %H:%M:%S]') $1\e[0m"
}

mkdir -p ${IMAGES_DIR}

IMAGE_NAME="clover_droneswarm.img"
IMAGE_PATH="${IMAGES_DIR}/${IMAGE_NAME}"

if [ ! -f "${IMAGE_PATH}" ]; then
    echo_stamp "Загрузка и распаковка..."
    wget -qO /tmp/clover.zip ${SOURCE_IMAGE}
    unzip -p /tmp/clover.zip *.img > ${IMAGE_PATH}
    rm /tmp/clover.zip
fi

img-resize ${IMAGE_PATH} max '6G'

echo_stamp "Копирование файлов DroneSwarm...."
DEV_IMAGE=$(losetup -Pf ${IMAGE_PATH} --show)
sleep 1
MOUNT_POINT=$(mktemp -d)
mount "${DEV_IMAGE}p2" ${MOUNT_POINT}

mkdir -p ${MOUNT_POINT}${TARGET_DIR}
cp -r ${REPO_DIR}/* ${MOUNT_POINT}${TARGET_DIR}/

chown -R 1000:1000 ${MOUNT_POINT}${TARGET_DIR}

umount -l ${MOUNT_POINT}
losetup -d ${DEV_IMAGE}

cat <<EOF > ${SCRIPTS_DIR}/install_deps.sh
#!/bin/bash
set -e
echo "Updating packages..."
apt-get update
# apt-get install -y example
cd ${TARGET_DIR}
# pip3 install -r requirements.txt
EOF

chmod +x ${SCRIPTS_DIR}/install_deps.sh

echo_stamp "Запуск установки образа..."
img-chroot ${IMAGE_PATH} exec ${TARGET_DIR}/builder/install_deps.sh

echo_stamp "Уменьшение образа..."
img-resize ${IMAGE_PATH}

echo_stamp "Создание прошло успешно: ${IMAGE_PATH}" "