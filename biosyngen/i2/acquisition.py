"""Versioned acquisition settings; sampling profiles do not qualify geometry or sync.

The legacy instrument constructors remain unchanged. The new profile samples
both observers afresh from the same trajectory, retaining independent clocks
and every other supplied sensor-stage parameter.
"""

from dataclasses import dataclass, replace

from .instruments import CableLptModel, DualPlateModel


@dataclass(frozen=True, slots=True)
class AcquisitionProfile:
    profile_id: str
    force_period_s: float
    lpt_period_s: float

    def instruments(
        self, lpt: CableLptModel, force: DualPlateModel | None = None,
    ) -> tuple[DualPlateModel, CableLptModel]:
        force = force or DualPlateModel()
        return (
            replace(force, clock=replace(force.clock, sample_period_s=self.force_period_s)),
            replace(lpt, clock=replace(lpt.clock, sample_period_s=self.lpt_period_s)),
        )


LEGACY_MIXED_RATE_V1 = AcquisitionProfile("BSMB-FORCE1000-LPT500-1.0", 0.001, 0.002)
DUAL_1KHZ_V1 = AcquisitionProfile("BSMB-FORCE1000-LPT1000-1.0", 0.001, 0.001)
