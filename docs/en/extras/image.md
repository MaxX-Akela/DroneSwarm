# Image for Raspberry Pi 4 / Orange Pi Pro

> 🚧 **The image for Technic 6S and Orange Pi 5 Pro is still in development.** Only the Clover image for Raspberry Pi 4 is ready for now. For other boards use [installation via apt](apt.md) or [Installation on Technic](technic_installation.md).

Ready-made images with DroneSwarm installed and configured are available in the [Releases](https://github.com/MaxX-Akela/DroneSwarm/releases) section. This is the easiest way to prepare a drone: see the [Quick Start](../fast_start.md).

## What is inside the image

The base is the official [Clover v0.25](https://github.com/CopterExpress/clover/releases/tag/v0.25) image on Raspbian Buster. The image is built automatically (GitHub Actions) for every release and differs from the original as follows:

* the DroneSwarm repository is in `/home/pi/DroneSwarm` together with the `.git` folder, so the drone reports a `branch@commit` version and updates through `git pull`;
* the `droneswarm` service (`/etc/systemd/system/droneswarm.service`) is enabled; it starts the client as user `pi` after Clover starts;
* `chrony` is installed with the drone config (`builder/assets/chrony-drone.conf`);
* the `drone-setup` command is added to `/usr/bin` for connecting to your router;
* the Raspbian source in `/etc/apt/sources.list` is replaced with `legacy.raspbian.org`, because the main repository for Buster no longer works.

Everything else (Clover, ROS, camera and flight controller settings) stays the same as in the original image. So the passwords, the drone's network name and SSH access match the [Clover](https://klever-doc.tech/ROS1/en/wifi.html) documentation.

## How to install

1. Download the `droneswarm_<version>.zip` archive from Releases and unpack it.
2. Write the `.img` to a MicroSD card with [balenaEtcher](https://etcher.balena.io).
3. Insert the card into the drone and continue from the "Starting the client" section of the [quick start](../fast_start.md).

## How to build the image yourself

You need Linux with the `qemu-user-static`, `binfmt-support`, `kpartx`, `wget` and `unzip` packages:

```bash
git clone https://github.com/MaxX-Akela/DroneSwarm
sudo bash DroneSwarm/builder/build.sh
```

The script downloads the Clover image, mounts it, copies the project in, installs `chrony` inside via `chroot` and puts the finished file into the `images/` folder. Run it from the folder that contains `DroneSwarm`.

> Building from source may lead to package version incompatibilities. If you are unsure, use the ready-made image.

## If the image does not boot

Install the client on top of the standard Clover image through [apt](apt.md).
