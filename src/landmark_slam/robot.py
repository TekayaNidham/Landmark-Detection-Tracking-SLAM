from __future__ import annotations

from dataclasses import dataclass, field
import random
from typing import List


@dataclass
class Robot:
    world_size: float = 100.0
    measurement_range: float = 30.0
    motion_noise: float = 1.0
    measurement_noise: float = 1.0
    x: float = 0.0
    y: float = 0.0
    landmarks: List[List[float]] = field(default_factory=list)
    num_landmarks: int = 0

    def __post_init__(self) -> None:
        self.x = self.world_size / 2.0
        self.y = self.world_size / 2.0

    def rand(self) -> float:
        return random.random() * 2.0 - 1.0

    def move(self, dx: float, dy: float) -> bool:
        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise

        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False

        self.x = x
        self.y = y
        return True

    def sense(self) -> List[List[float]]:
        measurements: List[List[float]] = []
        for i, landmark in enumerate(self.landmarks):
            dx = landmark[0] - self.x + self.rand() * self.measurement_noise
            dy = landmark[1] - self.y + self.rand() * self.measurement_noise
            if self.measurement_range == -1 or (abs(dx) <= self.measurement_range and abs(dy) <= self.measurement_range):
                measurements.append([i, dx, dy])
        return measurements

    def make_landmarks(self, num_landmarks: int) -> None:
        self.landmarks = [
            [round(random.random() * self.world_size), round(random.random() * self.world_size)]
            for _ in range(num_landmarks)
        ]
        self.num_landmarks = num_landmarks
