#!/usr/bin/env python3
"""Update this drone from git and restart the client service: `python3 update.py [--no-restart]`."""
import subprocess
import sys

from modules.version import REPO_DIR, get_version


def main():
    print("current:", get_version())
    result = subprocess.run(["git", "-c", "safe.directory=*", "pull", "--ff-only"], cwd=REPO_DIR)
    if result.returncode != 0:
        print("git pull failed, not restarting")
        return result.returncode
    print("updated:", get_version())
    if "--no-restart" not in sys.argv:
        return subprocess.run(["sudo", "systemctl", "restart", "droneswarm"]).returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
