# Installation via apt

For drones running the Clover image (Raspbian Buster) when no ready-made DroneSwarm image is available.

```bash
echo "deb [trusted=yes] https://maxx-akela.github.io/DroneSwarm/ ./" | sudo tee /etc/apt/sources.list.d/droneswarm.list
sudo apt update
sudo apt install drone-swarm
```

The `drone-swarm` package:
- installs `chrony` and replaces `/etc/chrony/chrony.conf` with the drone config (the original is saved as `chrony.conf.droneswarm-orig` and restored when the package is removed);
- puts the client in `/opt/droneswarm` and enables the `droneswarm` service;
- adds the `drone-setup` command for connecting to your router (see [Quick Start](../fast_start.md));
- is updated with `sudo apt update && sudo apt upgrade` or with the Update button on the server.

Without the repository: download `drone-swarm_*_all.deb` from [Releases](https://github.com/MaxX-Akela/DroneSwarm/releases)
and run `sudo apt install ./drone-swarm_*_all.deb`.

## Checking and removing

```bash
dpkg -l drone-swarm                # installed version
sudo systemctl status droneswarm   # client status
sudo apt remove drone-swarm        # removal: the service is disabled, chrony.conf is restored
```

After installing, connect the drone to the router and give it a name as described in the [quick start](../fast_start.md): `drone-setup <WIFI-SSID> <WIFI_PASS> <NAME>`.
