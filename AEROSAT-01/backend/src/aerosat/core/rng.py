"""Seed derivation, per-stream generators, Box-Muller.

ARCHITECTURE section 5.3: standard library only; one `random.Random` per stream; stream seed is
the first 8 bytes of SHA-256("{master_seed}:{stream_name}"); Gaussian values come from a
project-owned Box-Muller transform on `Random.random()`, because Python guarantees the
reproducibility of `random()` across versions but not of `gauss()`.
"""
import hashlib
import math
import random


def derive_seed(master_seed: int, stream_name: str) -> int:
    """Seed for one named stream: the first 8 bytes of the SHA-256 digest, read big-endian."""
    digest = hashlib.sha256(f"{master_seed}:{stream_name}".encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big")


class RandomStream:
    """An independent, reproducible random stream identified by a master seed and a name."""

    def __init__(self, master_seed: int, stream_name: str) -> None:
        self.stream_name = stream_name
        self._random = random.Random(derive_seed(master_seed, stream_name))

    def uniform(self) -> float:
        """Uniform value in [0, 1). Dimensionless."""
        return self._random.random()

    def standard_normal(self) -> float:
        """One draw from N(0, 1) by Box-Muller. Uses two uniform values per draw. Dimensionless."""
        u1 = 1.0 - self._random.random()  # in (0, 1], so the logarithm is finite
        u2 = self._random.random()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
