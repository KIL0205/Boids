"""sim — simulation-layer package.

Exports the main public types so callers can write:

    from sim import Simulation, Boid, Sardine, Sailfish, SpatialHash
"""

from .boid          import Boid
from .sardine       import Sardine
from .sailfish      import Sailfish
from .spatial_hash  import SpatialHash
from .simulation    import Simulation

__all__ = ["Boid", "Sardine", "Sailfish", "SpatialHash", "Simulation"]
