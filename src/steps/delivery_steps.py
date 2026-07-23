"""Steps that bridge missions and the delivery tracker service.

This file shows the canonical pattern for a *custom* step: subclass `Step`,
implement the async `_execute_step`, and reach shared state through
`robot.get_service(...)`. The thin `@dsl` factory at the bottom is what missions
call.
"""

from raccoon import GenericRobot, dsl
from raccoon.step import Step

from src.service.delivery_tracker_service import DeliveryTrackerService


@dsl(hidden=True)  # hidden=True keeps this out of the WebIDE palette (it's internal)
class RecordDeliveryStep(Step):
    """Increment the delivery counter held by DeliveryTrackerService."""

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # get_service lazily creates (and then caches) the service instance, so
        # every step that asks for DeliveryTrackerService shares the same state.
        tracker = robot.get_service(DeliveryTrackerService)
        tracker.record()


@dsl()
def record_delivery() -> RecordDeliveryStep:
    """Count one delivered object (logs the running total)."""
    return RecordDeliveryStep()
