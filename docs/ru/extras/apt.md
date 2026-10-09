# Установка через apt

Для дронов на образе Clover (Raspbian Buster), если нет готового образа DroneSwarm.

```bash
echo "deb [trusted=yes] https://maxx-akela.github.io/DroneSwarm/ ./" | sudo tee /etc/apt/sources.list.d/droneswarm.list
sudo apt update
sudo apt install drone-swarm
```

Пакет `drone-swarm`:
- ставит `chrony` и заменяет `/etc/chrony/chrony.conf` на конфиг дрона (оригинал сохраняется в `chrony.conf.droneswarm-orig` и возвращается при удалении пакета);
- кладёт клиент в `/opt/droneswarm` и включает сервис `droneswarm`;
- добавляет команду `drone-setup` для подключения к вашему роутеру (см. [Быстрый старт](../fast_start.md));
- обновляется через `sudo apt update && sudo apt upgrade` или кнопкой Update на сервере.

Без репозитория: скачайте `drone-swarm_*_all.deb` из [Releases](https://github.com/MaxX-Akela/DroneSwarm/releases)
и выполните `sudo apt install ./drone-swarm_*_all.deb`.

## Проверка и удаление

```bash
dpkg -l drone-swarm                # установленная версия
sudo systemctl status droneswarm   # состояние клиента
sudo apt remove drone-swarm        # удаление: сервис отключается, chrony.conf возвращается
```

После установки подключите дрон к роутеру и задайте ему имя, как описано в [быстром старте](../fast_start.md): `drone-setup <WIFI-SSID> <WIFI_PASS> <NAME>`.
