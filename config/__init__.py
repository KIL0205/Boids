"""config package — SimConfig dataclass, persistence, and runtime menu."""
from .config import SimConfig, CONFIG
from .config_io import load_overrides, save_config
from .config_menu import ConfigMenu

__all__ = ["SimConfig", "CONFIG", "load_overrides", "save_config", "ConfigMenu"]
