"""MuJoCo adapter: state, frames, contact/grip force ledger and plain exports.

Exports are plain Python data (no MuJoCo objects) so the independent
benchmark oracle can recompute balances without importing this package.
Sign conventions are documented, not inferred: a contact force is the
world-frame force on geom2's body (equal and opposite on geom1's body); a
connect force f is +f on site1's body at site1 and -f on site2's body.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

from .model import AthleteBarbellSpec, build_mjcf, joint_specs

SOURCE_OBSERVABLE = ("plate_force_world_n", "bar_position_world_m")
LATENT = ("joint_angles", "joint_torques", "grip_forces", "rack_contact_forces", "body_states")


@dataclass
class SpatialPlant:
    spec: AthleteBarbellSpec
    model: mujoco.MjModel
    data: mujoco.MjData

    @classmethod
    def build(cls, spec: AthleteBarbellSpec = AthleteBarbellSpec()):
        model = mujoco.MjModel.from_xml_string(build_mjcf(spec))
        return cls(spec, model, mujoco.MjData(model))

    # -- indices ---------------------------------------------------------
    def joint_qpos(self, name):
        return int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])

    def joint_dof(self, name):
        return int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])

    def body_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def site_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)

    @property
    def actuated(self):
        """(joint name, dof index, capacity) in actuator order."""
        return [(name, self.joint_dof(name), j.capacity_n_m) for name, j in joint_specs()]

    @property
    def free_dofs(self):
        return list(range(self.joint_dof("pelvis_free"), self.joint_dof("pelvis_free") + 6)) + \
            list(range(self.joint_dof("bar_free"), self.joint_dof("bar_free") + 6))

    # -- mechanics -------------------------------------------------------
    def total_mass(self):
        return float(self.model.body_mass.sum())

    def com(self):
        m = self.model.body_mass
        return (m[:, None] * self.data.xipos).sum(0) / m.sum()

    def body_velocity(self, body):
        """World linear velocity of the body COM and world angular velocity."""
        out = np.zeros(6)
        # For bodies MuJoCo reports [angular, linear] at the body COM (xipos),
        # world orientation; verified against mj_jac at xipos in the tests.
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, out, 0)
        return out[3:].copy(), out[:3].copy()

    def contact_ledger(self):
        rows = []
        force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            mujoco.mj_contactForce(self.model, self.data, i, force)
            frame = c.frame.reshape(3, 3)
            world = force[0] * frame[0] + force[1] * frame[1] + force[2] * frame[2]
            g1, g2 = int(c.geom1), int(c.geom2)
            rows.append(dict(
                geom1=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom{g1}",
                geom2=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom{g2}",
                body1=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[g1])),
                body2=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[g2])),
                point=c.pos.tolist(), normal=frame[0].tolist(), force_on_body2=world.tolist(),
                normal_force=float(force[0]), tangent_force=float(math.hypot(force[1], force[2])),
                friction=float(c.friction[0]), dist=float(c.dist)))
        return rows

    def grip_ledger(self):
        rows = []
        for e in range(self.model.neq):
            idx = [k for k in range(self.data.nefc)
                   if self.data.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_EQUALITY and self.data.efc_id[k] == e]
            f = np.array([self.data.efc_force[k] for k in idx]) if idx else np.zeros(3)
            s1, s2 = int(self.model.eq_obj1id[e]), int(self.model.eq_obj2id[e])
            rows.append(dict(name=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, e),
                             body1=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.site_bodyid[s1])),
                             body2=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.site_bodyid[s2])),
                             point1=self.data.site_xpos[s1].tolist(), point2=self.data.site_xpos[s2].tolist(),
                             force_on_body1=f.tolist(),
                             separation_m=float(np.linalg.norm(self.data.site_xpos[s1] - self.data.site_xpos[s2]))))
        return rows

    def export_frame(self):
        """Plain snapshot for the independent oracle (call after mj_forward)."""
        bodies = []
        for b in range(1, self.model.nbody):
            v, w = self.body_velocity(b)
            ipos_rot = self.data.ximat[b].reshape(3, 3)
            inertia_world = ipos_rot @ np.diag(self.model.body_inertia[b]) @ ipos_rot.T
            bodies.append(dict(name=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b),
                               mass=float(self.model.body_mass[b]), com=self.data.xipos[b].tolist(),
                               v=v.tolist(), w=w.tolist(), inertia_world=inertia_world.tolist()))
        return dict(t=float(self.data.time), bodies=bodies, contacts=self.contact_ledger(),
                    grips=self.grip_ledger(), external=[], gravity=self.model.opt.gravity.tolist())

    def joint_angles(self):
        return {name: float(self.data.qpos[self.joint_qpos(name)]) for name, _ in joint_specs()}
