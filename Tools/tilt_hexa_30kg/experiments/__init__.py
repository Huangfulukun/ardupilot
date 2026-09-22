# experiments/__init__.py -- Tilt-Hexa experiments package

"""Shared experiment-package compatibility hooks.

The paper campaigns analyse ArduPilot DataFlash ``.BIN`` files after each
SITL run.  ``pymavlink.mavutil.mavlink_connection`` is intended for MAVLink
streams/logs and can return no recognised records for DataFlash binaries.
Keep live MAVLink connections unchanged, but route DataFlash files through
``DFReader_binary`` so the existing metrics reader receives POS/ATT/THX*
messages from the actual flight log.
"""

import os

try:
    from pymavlink import mavutil
    from pymavlink.DFReader import DFReader_binary
except ImportError:
    mavutil = None
    DFReader_binary = None


if mavutil is not None and DFReader_binary is not None:
    _original_mavlink_connection = mavutil.mavlink_connection

    def _tilthexa_mavlink_connection(device, *args, **kwargs):
        try:
            path = os.fspath(device)
        except TypeError:
            path = None
        if isinstance(path, str) and path.lower().endswith(".bin"):
            return DFReader_binary(path)
        return _original_mavlink_connection(device, *args, **kwargs)

    # metrics_common.py historically opens post-flight logs through
    # mavutil.mavlink_connection().  Patch only .BIN paths; UDP/TCP/serial
    # links used by the SITL experiments continue to use mavutil unchanged.
    mavutil.mavlink_connection = _tilthexa_mavlink_connection
