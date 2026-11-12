"""RES-624 Gate E: independent angular-momentum oracle, convergence and fault injections.

The reference is a fourth-order Runge-Kutta integration of the rigid-body
attitude dynamics. The analytic trajectories here are produced by a *different*
integrator (SciPy DOP853 at rtol 1e-12), so the comparison is not self-
referential. Six deliberate faults must be rejected with a large margin, while
the torque-free, known-torque, contact-lever and world-frame cases pass.
"""
import math
import unittest

try:
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    np = None


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def _quat_to_R(q):
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def _R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1)*2
        q = [.25*s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1+R[0, 0]-R[1, 1]-R[2, 2])*2
        q = [(R[2, 1]-R[1, 2])/s, .25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s]
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1+R[1, 1]-R[0, 0]-R[2, 2])*2
        q = [(R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, .25*s, (R[1, 2]+R[2, 1])/s]
    else:
        s = np.sqrt(1+R[2, 2]-R[0, 0]-R[1, 1])*2
        q = [(R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, .25*s]
    q = np.array(q, float)
    return q/np.linalg.norm(q)


def _trajectory(D, R0, w_body0, torque_world, duration=.05, dt=.001):
    """DOP853 truth of q' = q (x) w_b/2, w_b' = D^-1(tau - w_b x D w_b)."""
    from scipy.integrate import solve_ivp
    D = np.asarray(D, float)
    tau_world = np.asarray(torque_world, float)

    def rhs(_t, y):
        q, wb = y[:4], y[4:]
        tau_body = _quat_to_R(q).T@tau_world
        dwb = (tau_body-np.cross(wb, D*wb))/D
        return np.concatenate([.5*_quat_mul(q, np.array([0., *wb])), dwb])
    y0 = np.concatenate([_R_to_quat(np.asarray(R0, float)), np.asarray(w_body0, float)])
    n = int(round(duration/dt))
    times = np.linspace(0., duration, n+1)
    sol = solve_ivp(rhs, (0., duration), y0, t_eval=times, method="DOP853", rtol=1e-12, atol=1e-14)
    states = []
    for k, t in enumerate(times):
        q = sol.y[:4, k]
        q = q/np.linalg.norm(q)
        R = _quat_to_R(q)
        wb = sol.y[4:, k]
        states.append((float(t), R, R@wb))
    return states


def _frames(states, D, *, torque_couples=None, contact_couples=None):
    frames = []
    for t, R, w in states:
        frame = dict(t=t, gravity=[0., 0., 0.],
                     bodies=[dict(name="barbell", mass=20., com=[0., 0., 0.], v=[0., 0., 0.],
                                  w=w.tolist(), inertia_world=(R@np.diag(D)@R.T).tolist())],
                     bar_frame=dict(position=[0., 0., 0.], rotation=R.tolist(), axis=R[:, 1].tolist()),
                     contacts=[], grips=[], external=[])
        if torque_couples:
            frame["external"] = [dict(body="barbell", point=p, force=f) for p, f in torque_couples]
        if contact_couples:
            frame["contacts"] = [dict(body1="barbell", body2="hand", geom1="bar_shaft", geom2="l_finger_distal_pad",
                                      point=p, normal=[0., 1., 0.], force_on_body2=(-np.asarray(f)).tolist(),
                                      normal_force=float(np.linalg.norm(f)), tangent_force=0., friction=1., dist=0.)
                                 for p, f in contact_couples]
        frames.append(frame)
    return frames


def _couple(torque):
    """Two forces at +/- 0.5 m on x that produce the given torque about z."""
    assert abs(torque[0]) < 1e-15 and abs(torque[1]) < 1e-15
    f = [0., float(torque[2]), 0.]
    return [([.5, 0., 0.], f), ([-.5, 0., 0.], [-f[0], -f[1], -f[2]])]


def _rotate(R, vecs):
    return [ (R@np.asarray(v, float)).tolist() for v in vecs ]


@unittest.skipIf(np is None, "NumPy/SciPy required")
class AnalyticAngularOracle(unittest.TestCase):
    # Plain lists: the stdlib CI imports this module without NumPy and converts them in the tests.
    D = [.08, .05, .03]
    R0 = [[.36, -.48, .8], [.8, .36, -.48], [-.48, -.8, -.36]]
    W0 = [2.0, .5, 1.0]

    def run_check(self, frames, tolerance=1e-7):
        from biosyngen_benchmark.rigid_body_check import angular_reference_check
        return angular_reference_check(frames, .001, tolerance_n_m_s=tolerance)

    def test_torque_free_rotation_passes(self):
        frames = _frames(_trajectory(self.D, self.R0, self.W0, np.zeros(3)), self.D)
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "PASS", r)
        self.assertLess(r["max_residual_n_m_s"], 1e-10)

    def test_known_external_torque_passes(self):
        torque = np.array([0., 0., 1.])
        states = _trajectory(self.D, self.R0, self.W0, torque)
        frames = _frames(states, self.D, torque_couples=_couple(torque))
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "PASS", r)
        self.assertLess(r["max_residual_n_m_s"], 1e-10)

    def test_contact_lever_arm_moment_passes_and_corrupt_point_is_rejected(self):
        torque = np.array([0., 0., 1.])
        states = _trajectory(self.D, self.R0, self.W0, torque)
        frames = _frames(states, self.D, contact_couples=_couple(torque))
        self.assertEqual(self.run_check(frames)["outcome"], "PASS")
        corrupted = []
        for f in frames:
            contacts = [dict(c) for c in f["contacts"]]
            contacts[0]["point"] = [contacts[0]["point"][0]+.01, contacts[0]["point"][1], contacts[0]["point"][2]]
            corrupted.append(dict(f, contacts=contacts))
        r = self.run_check(corrupted)
        self.assertEqual(r["outcome"], "REJECTED")
        self.assertGreater(r["max_residual_n_m_s"]/r["tolerance_n_m_s"], 10.)

    def test_world_frame_rotation_invariance(self):
        torque = np.array([0., 0., 1.])
        states = _trajectory(self.D, self.R0, self.W0, torque)
        frames = _frames(states, self.D, torque_couples=_couple(torque))
        Q = _quat_to_R(_R_to_quat(np.array([[.3, -.5, .81], [.9, .4, -.2], [-.3, .77, .56]])))
        rotated = []
        for f in frames:
            b = f["bodies"][0]
            rotated.append(dict(f,
                bodies=[dict(b, w=(Q@np.asarray(b["w"])).tolist(),
                             inertia_world=(Q@np.asarray(b["inertia_world"])@Q.T).tolist())],
                bar_frame=dict(f["bar_frame"], rotation=(Q@np.asarray(f["bar_frame"]["rotation"])).tolist(),
                               axis=(Q@np.asarray(f["bar_frame"]["axis"])).tolist()),
                external=[dict(e, point=(Q@np.asarray(e["point"])).tolist(), force=(Q@np.asarray(e["force"])).tolist())
                          for e in f["external"]]))
        a = self.run_check(frames)
        b = self.run_check(rotated)
        self.assertEqual(b["outcome"], "PASS", b)
        self.assertAlmostEqual(a["max_residual_n_m_s"], b["max_residual_n_m_s"], delta=1e-12)

    def test_removed_torque_is_rejected(self):
        torque = np.array([0., 0., 1.])
        frames = _frames(_trajectory(self.D, self.R0, self.W0, torque), self.D, torque_couples=_couple(torque))
        for f in frames:
            f["external"] = []
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "REJECTED")
        self.assertGreater(r["max_residual_n_m_s"], .5*1e-3)

    def test_changed_inertia_is_rejected(self):
        torque = np.array([0., 0., 1.])
        frames = _frames(_trajectory(self.D, self.R0, self.W0, torque), self.D, torque_couples=_couple(torque))
        for f in frames:
            f["bodies"][0]["inertia_world"] = (1.2*np.asarray(f["bodies"][0]["inertia_world"])).tolist()
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "REJECTED")

    def test_reversed_contact_moment_is_rejected(self):
        torque = np.array([0., 0., 1.])
        frames = _frames(_trajectory(self.D, self.R0, self.W0, torque), self.D, contact_couples=_couple(torque))
        for f in frames:
            for c in f["contacts"]:
                c["force_on_body2"] = (-np.asarray(c["force_on_body2"])).tolist()
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "REJECTED")

    def test_undeclared_angular_impulse_is_rejected(self):
        frames = _frames(_trajectory(self.D, self.R0, self.W0, np.zeros(3)), self.D)
        frames[25]["bodies"][0]["w"] = (np.asarray(frames[25]["bodies"][0]["w"])+np.array([0., 0., .01])).tolist()
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "REJECTED")
        self.assertGreater(r["max_residual_n_m_s"]/r["tolerance_n_m_s"], 10.)

    def test_altered_angular_velocity_is_rejected(self):
        torque = np.array([0., 0., 1.])
        frames = _frames(_trajectory(self.D, self.R0, self.W0, torque), self.D, torque_couples=_couple(torque))
        for f in frames:
            f["bodies"][0]["w"] = (1.01*np.asarray(f["bodies"][0]["w"])).tolist()
        r = self.run_check(frames)
        self.assertEqual(r["outcome"], "REJECTED")
        self.assertGreater(r["max_residual_n_m_s"]/r["tolerance_n_m_s"], 10.)


@unittest.skipIf(np is None, "NumPy/SciPy required")
class NominalAngularConvergence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import mujoco  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))
        from scripts.res624_receiving_campaign import build_nominal, run_case
        _, cls.nominal = build_nominal()
        cls.frames, cls.static, cls.plant = run_case("nominal_0p3s_red_window", None, None, .3, {}, cls.nominal)

    def coarse(self):
        from biosyngen_benchmark.rigid_body_check import angular_reference_check
        window = [f for f in self.frames if f["t"] >= .1]
        return angular_reference_check(window, self.plant.model.opt.timestep)

    def test_incorrect_bar_inertia_is_rejected_on_the_real_hold(self):
        import copy
        from biosyngen_benchmark.rigid_body_check import angular_reference_check
        frames = copy.deepcopy([f for f in self.frames if f["t"] >= .1])
        for f in frames:
            body = next(b for b in f["bodies"] if b["name"] == "barbell")
            body["inertia_world"] = (1.2*np.asarray(body["inertia_world"])).tolist()
        r = angular_reference_check(frames, self.plant.model.opt.timestep)
        self.assertEqual(r["outcome"], "REJECTED")
        self.assertGreater(r["max_residual_n_m_s"]/r["tolerance_n_m_s"], 10.)

    def test_nominal_reference_passes_and_refines_with_dt(self):
        import mujoco
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        from biosyngen.spatial.model_v2 import HumanV2Spec
        from biosyngen.spatial.receiving_grasp_v2 import forward_receiving_hold, qualify_static
        from biosyngen_benchmark.rigid_body_check import angular_reference_check
        coarse = self.coarse()
        self.assertEqual(coarse["outcome"], "PASS", coarse)
        plant = HumanV2Plant.build(HumanV2Spec())
        plant.model.opt.timestep = 5e-4
        plant.data.qpos[:] = self.nominal["qpos"]
        mujoco.mj_forward(plant.model, plant.data)
        static = qualify_static(plant)
        frames = forward_receiving_hold(plant, static["certificate"], .3)
        fine = angular_reference_check([f for f in frames if f["t"] >= .1], 5e-4)
        self.assertEqual(fine["outcome"], "PASS", fine)
        self.assertLess(fine["max_residual_n_m_s"], coarse["max_residual_n_m_s"])
        self.assertLess(coarse["max_residual_n_m_s"], coarse["tolerance_n_m_s"])


if __name__ == "__main__":
    unittest.main()
