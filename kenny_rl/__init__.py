"""KENNY's lightweight navigation simulator; no ROS or GPU required."""
__all__ = ["KennyEnv"]


def __getattr__(name):
    # Keep CLI startup free of NumPy imports until worker thread limits are set.
    if name == "KennyEnv":
        from .env import KennyEnv
        return KennyEnv
    raise AttributeError(name)
