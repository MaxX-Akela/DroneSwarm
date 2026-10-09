# Образ для Raspberry Pi 4 / Orange Pi Pro

> 🚧 **Образ для Technic 6S и Orange Pi 5 Pro пока в разработке.** Сейчас готов только образ для Clover на Raspberry Pi 4. Для остальных плат пользуйтесь [установкой через apt](apt.md) или [Установкой на Technic](technic_installation.md).

Готовые образы с установленным и настроенным DroneSwarm лежат в разделе [Releases](https://github.com/MaxX-Akela/DroneSwarm/releases). Это самый простой способ подготовить дрон: см. [Быстрый старт](../fast_start.md).

## Что внутри образа

За основу взят официальный образ [Clover v0.25](https://github.com/CopterExpress/clover/releases/tag/v0.25) на Raspbian Buster. Образ собирается автоматически (GitHub Actions) при создании каждого релиза и отличается от оригинала так:

* репозиторий DroneSwarm лежит в `/home/pi/DroneSwarm` вместе с папкой `.git`, поэтому дрон показывает версию вида `ветка@коммит` и обновляется через `git pull`;
* включён сервис `droneswarm` (`/etc/systemd/system/droneswarm.service`), который запускает клиент от пользователя `pi` после старта Clover;
* установлен `chrony` с конфигом дрона (`builder/assets/chrony-drone.conf`);
* в `/usr/bin` добавлена команда `drone-setup` для подключения к вашему роутеру;
* в `/etc/apt/sources.list` источник Raspbian заменён на `legacy.raspbian.org`, потому что основной репозиторий для Buster больше не работает.

Всё остальное — Clover, ROS, настройки камеры и полётного контроллера — остаётся таким же, как в оригинальном образе. Поэтому пароли, имя сети дрона и подключение по SSH совпадают с документацией [Clover](https://klever-doc.tech/ROS1/ru/wifi.html).

## Как установить

1. Скачайте архив `droneswarm_<версия>.zip` из Releases и распакуйте его.
2. Запишите `.img` на карту MicroSD через [balenaEtcher](https://etcher.balena.io).
3. Вставьте карту в дрон и продолжайте с пункта «Запуск клиента» в [быстром старте](../fast_start.md).

## Как собрать образ самому

Нужен Linux с пакетами `qemu-user-static`, `binfmt-support`, `kpartx`, `wget`, `unzip`:

```bash
git clone https://github.com/MaxX-Akela/DroneSwarm
sudo bash DroneSwarm/builder/build.sh
```

Скрипт скачает образ Clover, смонтирует его, скопирует проект, установит `chrony` внутри через `chroot` и положит готовый файл в папку `images/`. Запускайте его из папки, рядом с которой лежит `DroneSwarm`.

> Сборка из исходников может привести к несовместимости версий пакетов. Если вы не уверены, используйте готовый образ.

## Если образ не запускается

Поставьте клиент поверх стандартного образа Clover через [apt](apt.md).
