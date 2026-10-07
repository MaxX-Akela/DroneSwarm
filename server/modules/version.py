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


def get_version(fallback="unknown"):
    """'<branch>@<short commit id>' of the checkout this code runs from.

    A trailing '*' marks tracked files modified on top of that commit.
    """
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
    """git fetch + git pull --rebase. Returns (ok, output)."""
    output = []
    for args in (("fetch",), ("pull", "--rebase")):
        try:
            result = subprocess.run(
                ["git", "-c", "safe.directory=*", *args],
                cwd=REPO_DIR, capture_output=True, text=True, timeout=60,
            )
        except (OSError, subprocess.SubprocessError) as e:
            return False, str(e)
        output.append((result.stdout + result.stderr).strip())
        if result.returncode != 0:
            return False, "\n".join(output)
    return True, "\n".join(output)
