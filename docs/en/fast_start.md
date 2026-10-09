# Quick Start

## Required equipment

* A set of drones running [PX4](https://px4.io) firmware with [Clover](https://github.com/CopterExpress/clover) or [Technic](https://docs.skyris.ru/technic6S/) images
* A computer running Windows or Ubuntu, or the [Clover simulator](https://klever-doc.tech/ROS1/en/simulation.html)
> We recommend Ubuntu or the simulator: everything works there without problems.
* A Wi-Fi router supporting 2.4 and 5 GHz
> The main thing is that your devices support 5 GHz.

## Preparing the hardware and software

DroneSwarm consists of two parts: the [server]() and the [drone (client)](). Let's look at the drone part in more detail. There are three ways to get it running:

1. **Ready-made image.** The simplest option: download the latest release for your companion board.

   > 🚧 The image for Technic 6S and Orange Pi 5 Pro is still in development. For them, use method 2 or 3 (some rough edges are possible).

2. **Installation via apt.** Slightly harder, see [this guide](/en/extras/apt.md).
3. **Building from source.** Instructions for each platform: [Clover](/en/extras/clover_installation.md), [Technic](/en/extras/technic_installation.md).

> We recommend method 1 for most users. If the image from the releases does not boot, use method 2.

Download the image from the Releases section on GitHub and install [balenaEtcher](https://etcher.balena.io). Download the server source code to your computer with `git clone` or as a ZIP archive and unpack it.

## Starting the client

1. Write the downloaded image to a MicroSD card with Etcher.
2. Insert the card into the Raspberry Pi 4 or Orange Pi 5 Pro on the drone, power the drone on and wait for the `clever-xxxx` or `technic-xxxx` network to appear.

   > When powering the drone from a battery outside the flight area, remove the propellers or use another power source.

   > Raspberry Pi 4 and Orange Pi 5 Pro are sensitive to power. Use a powerful power supply over USB-C.

3. Connect to the drone's network using the image's password: [Clover](https://klever-doc.tech/ROS1/en/wifi.html), [Technic](https://docs.skyris.ru/technic6S/ConnectingToWi-Fi.html).
4. Connect to the microcomputer over SSH. The connection details differ between drone brands: [Clover](https://klever-doc.tech/ROS1/en/ssh.html), [Technic](https://docs.skyris.ru/technic6S/SSH.html).
5. Run the script that connects the drone to your router and changes the `hostname`:

   > `hostname` is the drone's name in the table on the server.

```bash
   drone-setup <WIFI-SSID> <WIFI_PASS> <NAME>
```

   Example:
```bash
   drone-setup Keenetic-8989 iloverclover4 clover-1
```
   (for Technic use a name like `technic-1`)

The drones should now connect to your router. If they don't, see [Troubleshooting and FAQ](troubleshooting.md).

**More about the client: [extras/_drone.md](extras/_drone.md)**
