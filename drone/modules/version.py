import logging
import os
import subprocess

logger = logging.getLogger(__name__)

REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
GIT_TIMEOUT = 5.0


def _git(*args):
    return subprocess.run(
        ["git", "-c", "safe.directory=*", *args],
        cwd=REPO_DIR, capture_output=True, text=True, timeout=GIT_TIMEOUT, check=True,
    ).stdout.strip()


PACKAGE = "drone-swarm"


def _package_version():
    """Version of the installed .deb, or None when running from a git checkout."""
    try:
        out = subprocess.run(
            ["dpkg-query", "-W", "-f=${Version}", PACKAGE],
            capture_output=True, text=True, timeout=GIT_TIMEOUT, check=True,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None
    return out or None


def get_version(fallback="unknown"):
    """'<branch>@<short commit id>' of the checkout this code runs from.

    For an apt install (no .git) it is the package version instead.

    A trailing '*' marks tracked files modified on top of that commit.
    """
    if not os.path.exists(os.path.join(REPO_DIR, ".git")):
        return _package_version() or fallback
    try:
        sha = _git("rev-parse", "--short", "HEAD")
        branch = _git("rev-parse", "--abbrev-ref", "HEAD")
        if branch == "HEAD":
            try:
                branch = _git("describe", "--tags", "--exact-match")
            except subprocess.CalledProcessError:
                branch = "detached"
        dirty = bool(_git("status", "--porcelain", "--untracked-files=no"))
    except (OSError, subprocess.SubprocessError) as e:
        logger.debug("git version unavailable: %s", e)
        return fallback
    return f"{branch}@{sha}{'*' if dirty else ''}"


def git_pull():
    """Fast-forward the checkout. Returns (ok, output)."""
    try:
        result = subprocess.run(
            ["git", "-c", "safe.directory=*", "pull", "--ff-only"],
            cwd=REPO_DIR, capture_output=True, text=True, timeout=60,
        )
    except (OSError, subprocess.SubprocessError) as e:
        return False, str(e)
    return result.returncode == 0, (result.stdout + result.stderr).strip()
