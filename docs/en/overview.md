# Introduction

## Why DroneSwarm exists

Anyone who has worked with COEX drones for a while knows that `clever-show` was the original swarm software for Clover. It is a good project, and I like it myself.

But in October–November 2025 Raspbian Buster went legacy and its repositories were archived. Since then it has been impossible to install `clever-show` following the [official guide](https://github.com/CopterExpress/clever-show/blob/master/docs/en/clover_installation.md) without editing `/etc/apt/sources.list` or building `chrony` from source. I even opened an issue about it, but closed it once I figured out how to install `clever-show` myself.

That story pushed me to make my own project. It is inspired by `clever-show`: I took the ideas that worked well and added my own.
