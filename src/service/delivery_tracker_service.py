"""A minimal stateful service, to show the third layer of the architecture.

Layers:  mission (what to do)  →  step (one action)  →  service (shared state).

A `RobotService` is the right home for state and logic that must live longer than
a single step and be shared across steps/missions — here, "how many objects have
we delivered?". Services are looked up with `robot.get_service(ServiceClass)`,
which creates the instance on first use and reuses it thereafter.

Because `RobotService` extends `ClassNameLogger`, calling `self.info(...)` tags
each log record with the class + method (`DeliveryTrackerService.record`) in the
JSONL log's `func` field — no manual prefixing needed. See docs/LOGGING.md.
"""

from raccoon import GenericRobot, RobotService

# The example task delivers a single object; a real game would set the real goal.
TARGET_DELIVERIES = 1


class DeliveryTrackerService(RobotService):
    """Counts delivered objects and logs progress toward the target."""

    def __init__(self, robot: "GenericRobot") -> None:
        super().__init__(robot)
        self._delivered = 0

    @property
    def delivered(self) -> int:
        return self._delivered

    def record(self) -> None:
        """Register one successful delivery and log the running total."""
        self._delivered += 1
        self.info(f"Delivered {self._delivered}/{TARGET_DELIVERIES} objects")

        if self._delivered >= TARGET_DELIVERIES:
            # info() for milestones, warn()/error() for problems. All levels land
            # in the JSONL file; only warn/error also print to the console.
            self.info("All target objects delivered 🎉")
