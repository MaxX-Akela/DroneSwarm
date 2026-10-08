#!/usr/bin/env python3
"""Update this drone and restart the client service: `python3 update.py [--no-restart]`.

Git checkouts are updated with `git pull`, apt installs with `apt-get install --only-upgrade`.
"""
import os
import subprocess
import sys

from modules.version import PACKAGE, REPO_DIR, get_version


def _update_git():
    return subprocess.run(["git", "-c", "safe.directory=*", "pull", "--ff-only"], cwd=REPO_DIR).returncode


def _update_apt():
    # DRONESWARM_NO_RESTART: the package must not restart the service under us; we do it below.
    cmds = [
        ["sudo", "-n", "apt-get", "update"],
        ["sudo", "-n", "env", "DRONESWARM_NO_RESTART=1", "apt-get", "install", "-y", "--only-upgrade", PACKAGE],
    ]
    for cmd in cmds:
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            return rc
    return 0


def main():
    print("current:", get_version())
    use_git = os.path.exists(os.path.join(REPO_DIR, ".git"))
    rc = _update_git() if use_git else _update_apt()
    if rc != 0:
        print("update failed, not restarting")
        return rc
    print("updated:", get_version())
    if "--no-restart" not in sys.argv:
        return subprocess.run(["sudo", "systemctl", "restart", "droneswarm"]).returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
