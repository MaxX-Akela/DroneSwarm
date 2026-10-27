import logging
import os

from modules.config import config

logger = logging.getLogger(__name__)

LINK = '\t<li><a href="swarm.html">View Swarm visualization</a> (DroneSwarm server)</li>\n'
# Clover's menu entry we append after.
ANCHOR = "3D visualization for markers map"
REDIRECT = ('<!DOCTYPE html>\n<meta charset="utf-8">\n<title>Swarm visualization</title>\n'
            '<script>location.replace("http://{ip}:{port}/");</script>\n'
            '<a href="http://{ip}:{port}/">Swarm visualization</a>\n')


def _write_if_changed(path, text):
    try:
        with open(path, "r", encoding="utf-8") as f:
            if f.read() == text:
                return
    except OSError:
        pass
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def install_swarm_link(server_ip, web_port):
    """Add "View Swarm visualization" to the Clover web menu; swarm.html redirects to the server."""
    www = config.web_clover_www
    index = os.path.join(www, "index.html")
    if not web_port or not os.path.isfile(index):
        return False
    try:
        _write_if_changed(os.path.join(www, "swarm.html"), REDIRECT.format(ip=server_ip, port=web_port))
        with open(index, "r", encoding="utf-8") as f:
            lines = f.readlines()
        if any("swarm.html" in line for line in lines):
            return True
        for i, line in enumerate(lines):
            if ANCHOR in line:
                lines.insert(i + 1, LINK)
                break
        else:
            logger.warning("Clover menu entry not found in %s, swarm link not added", index)
            return False
        with open(index, "w", encoding="utf-8") as f:
            f.writelines(lines)
        logger.info("Swarm visualization link added to %s", index)
        return True
    except OSError as e:
        logger.warning("Can't add swarm link to the Clover web: %s", e)
        return False
