"""revohuman_bringup: only launch files and parameter defaults, **no node code**.

The nodes are implemented in `revohuman_ros`; this package exists to give the stack a **single entry
point**, so that "which parameters to set when starting it" has exactly one answer
(`config/revohuman.yaml`) instead of being scattered across everyone's shell history.
"""
