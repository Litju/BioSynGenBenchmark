"""Versioned V2 state/actuation/export interface, independent of V1 controllers."""
import mujoco
import numpy as np

from .model_v2 import HumanV2Spec, build_mjcf_v2, MODEL_ID, MODEL_VERSION, CONTACT_POLICY
from .plant import SpatialPlant


class HumanV2Plant(SpatialPlant):
    @classmethod
    def build(cls, spec=HumanV2Spec()):
        m = mujoco.MjModel.from_xml_string(build_mjcf_v2(spec))
        plant = cls(spec, m, mujoco.MjData(m))
        mujoco.mj_forward(m, plant.data)
        return plant

    @property
    def actuated(self):
        m = self.model
        return [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(j)), int(m.jnt_dofadr[j]),
                 float(m.actuator_forcerange[a, 1])) for a, j in enumerate(m.actuator_trnid[:, 0])]

    def grip_ledger(self):
        # The four equality constraints are internal scapular rhythm, never grip.
        return []

    def joint_angles(self):
        m, d = self.model, self.data
        return {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): float(d.qpos[m.jnt_qposadr[j]])
                for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE}

    def set_targets(self, targets, feedforward=None):
        """Bounded servo/motor control, no root wrench or hidden stabilization.

        Inherited general actuators integrate damping implicitly. New motors
        use explicit PD; their work and saturation remain in engine exports.
        """
        m, d = self.model, self.data
        known = self.joint_angles()
        if set(targets)-set(known) or (feedforward is not None and set(feedforward)-set(known)):
            raise ValueError("unknown joint target/feedforward")
        for name, value in targets.items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if not np.isfinite(value) or not m.jnt_range[j, 0] <= value <= m.jnt_range[j, 1]:
                raise ValueError(f"target outside declared ROM: {name}")
        feedforward = feedforward or {}
        if any(not np.isfinite(v) for v in feedforward.values()):
            raise ValueError("nonfinite feedforward")
        for a, (name, dof, capacity) in enumerate(self.actuated):
            joint = m.actuator_trnid[a, 0]
            q = d.qpos[m.jnt_qposadr[joint]]
            target = targets.get(name, q)
            ff = feedforward.get(name, 0.)
            if m.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_AFFINE:
                d.ctrl[a] = target + ff/m.actuator_gainprm[a, 0]
            else:
                d.ctrl[a] = np.clip(10*capacity*(target-q)-.25*capacity*d.qvel[dof]+ff, -capacity, capacity)

    def export_frame(self):
        frame = super().export_frame()
        frame.update(model_id=MODEL_ID, model_version=MODEL_VERSION, contact_policy=CONTACT_POLICY,
                     joint_angles=self.joint_angles(),
                     actuator_force_nm=self.data.actuator_force.tolist(),
                     actuator_velocity_rad_s=self.data.actuator_velocity.tolist(),
                     actuator_power_w=float(self.data.actuator_force @ self.data.actuator_velocity),
                     saturation_count=int(np.count_nonzero(np.abs(self.data.actuator_force) >= self.model.actuator_forcerange[:, 1]*(1-1e-9))))
        return frame
