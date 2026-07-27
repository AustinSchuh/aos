"""Picks the matplotlib backend to draw with on this platform.

We select explicitly rather than letting matplotlib work it out, because its
automatic detection has picked badly for us on Linux before.

Call use_interactive() before importing matplotlib.pyplot.  Scripts which only
generate code and never draw do not need to call it at all.
"""

import sys

import matplotlib


def use_interactive():
    """Configures matplotlib to draw in a window on this platform."""
    if sys.platform == "darwin":
        # OSX has no GTK.  matplotlib ships its own Cocoa backend, and the
        # native half of it comes prebuilt in the wheel.
        matplotlib.use("MacOSX")
    else:
        matplotlib.use("GTK3Agg")
