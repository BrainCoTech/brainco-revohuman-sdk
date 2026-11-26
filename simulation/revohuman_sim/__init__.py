"""RevoHuman glove MuJoCo visualization: dual-hand forward kinematics + Tracker 6D pose + tactile heatmap.

The entry point is simulation/viewer.py (or simulation/run_viewer.sh). This package is not installed with the
SDK; it runs only from the simulation/ directory of the source bundle. It does not import the SDK and gets its
data only by subscribing to the SDK's local broadcast (``revohuman-live``);
see live_client.py for the protocol.
"""
