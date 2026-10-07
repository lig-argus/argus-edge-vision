"""YOLOX output data only; C++ owns the public Observation/Track contracts."""
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

    def __post_init__(self):
        if not 0 <= self.confidence <= 1 or self.x2 < self.x1 or self.y2 < self.y1:
            raise ValueError("invalid full box")
