"""RES-624 fabricated false-support scenes for the independent evaluator.

These scenes start from the genuine nominal frame export and reallocate the
*bar-side wrench* between the permitted grasp interfaces and other bodies
while keeping, exactly:

* the net force and moment on the barbell (the bar's Newton-Euler balance),
* the whole-system force and moment balance (the floor reactions are solved
  for the difference), and
* contact admissibility of every modified contact (scaled forces stay inside
  their contact cones).

They are deliberately *ledger* attacks: no pose, trajectory, controller or
contact coefficient is changed. The only thing that changes is which body is
claimed to carry the load. That is exactly the false-acceptance surface the
receiving evaluator must refuse, and it is stronger than deleting a contact
(which already opens the bar balance).

NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from biosyngen_benchmark.grasp_contract import BAR_BODY, classify_frame

WORLD = "world"
SEGMENTS = {
    "forearm": ("l_forearm", "r_forearm", "l_forearm_collision", "r_forearm_collision"),
    "thigh": ("l_thigh", "r_thigh", "l_thigh_collision", "r_thigh_collision"),
    "torso": ("lumbar", "lumbar", "lumbar_collision", "lumbar_collision"),
}


def _bar_force(contact, bar=BAR_BODY):
    f = np.asarray(contact["force_on_body2"], float)
    return (f if contact["body2"] == bar else -f), np.asarray(contact["point"], float)


def _restore_system_balance(frames, bar=BAR_BODY):
    """Remove the net system force of the added external bar loads with floor reactions.

    The evaluator gates the whole-system *linear* momentum balance. The
    existing foot reactions are scaled uniformly (which preserves every
    contact's friction-cone ratio) and the small remaining horizontal force is
    carried by one declared heel-region foot contact with enough normal force
    to stay inside its cone. Every modified contact is checked explicitly.
    (The scene is a ledger-level false-support fabrication: it preserves the
    bar wrench, the system force balance, contact admissibility, the support
    polygon and the static verdict, and differs only in which body carries the
    bar. The system angular balance is reported by the evaluator but, as in the
    pre-repair evaluator, is not a gating condition; see the review package.)
    """
    for frame in frames[:-1]:
        added = [e for e in frame.get("external", ()) if e["body"] == bar]
        if not added:
            continue
        f_ext = sum((np.asarray(e["force"], float) for e in added), np.zeros(3))
        floor = [(i, c) for i, c in enumerate(frame["contacts"]) if WORLD in (c["body1"], c["body2"])]
        if not floor:
            raise ValueError("false-support scene needs floor contacts to keep the system balance closed")
        f_floor = sum((np.asarray(c["force_on_body2"], float)*(1 if c["body2"] != WORLD else -1) for _, c in floor),
                      np.zeros(3))
        if abs(f_floor[2]) < 1e-9:
            raise ValueError("false-support scene has no floor vertical support to redistribute")
        mu = float(floor[0][1]["friction"])
        beta = (f_floor[2]-f_ext[2])/f_floor[2]
        for _ in range(8):
            residual = (f_floor - f_ext) - beta*f_floor
            need = 1.05*float(np.linalg.norm(residual[:2]))/mu
            if residual[2] >= need:
                break
            beta = (f_floor[2]-f_ext[2]-need)/f_floor[2]
        if beta <= 0:
            raise ValueError("false-support scene would remove all floor support")
        for _, c in floor:
            c["force_on_body2"] = (beta*np.asarray(c["force_on_body2"], float)).tolist()
        residual = (f_floor - f_ext) - beta*f_floor
        point = np.mean([np.asarray(c["point"], float) for _, c in floor], axis=0)
        point[2] = 0.
        frame["contacts"].append(dict(
            geom1="floor", geom2="l_heel_sole", body1=WORLD, body2="l_foot",
            point=point.tolist(), normal=[0., 0., 1.],
            force_on_body2=[float(residual[0]), float(residual[1]), float(residual[2])],
            normal_force=float(residual[2]), tangent_force=float(np.linalg.norm(residual[:2])), friction=mu, dist=0.))
        for index, c in floor:
            f_on_foot = np.asarray(c["force_on_body2"], float)*(1 if c["body2"] != WORLD else -1)
            fn = float(f_on_foot @ np.asarray(c["normal"], float))
            if fn < 0:
                raise ValueError(f"false-support scene would make floor contact {index} adhesive")
            ft = float(np.linalg.norm(f_on_foot-fn*np.asarray(c["normal"], float)))
            if ft > c["friction"]*fn*(1+1e-6):
                raise ValueError(f"false-support scene would violate the floor cone at contact {index}")
        added_contact = frame["contacts"][-1]
        if np.linalg.norm(added_contact["force_on_body2"][:2]) > mu*added_contact["force_on_body2"][2]:
            raise ValueError("false-support scene heel contact violates its own cone")


def redistribute_hand_load(frames, factors, bar=BAR_BODY):
    """Scale each side's permitted hand contact forces, compensating externally.

    ``factors`` maps ``"l"``/``"r"`` to the multiplier applied to every contact
    force of that side's permitted grasp contacts. The bar-side wrench is kept
    exact by adding the complement as a *declared* external load at the same
    contact points; the whole-system balance is then restored by the floor
    reactions. A factor of 1.0/0.6/1.4 is therefore an asymmetric 30/70 hand
    load sharing whose real support still comes entirely from the hands.
    """
    out = copy.deepcopy(frames)
    for frame in out:
        ledger = classify_frame(frame, bar)
        extra = []
        for row in ledger["contacts"]:
            factor = float(factors.get(row["side"], 1.0))
            if abs(factor-1.0) < 1e-15:
                continue
            contact = frame["contacts"][row["index"]]
            force_bar = np.asarray(row["force_on_bar"], float)
            contact["force_on_body2"] = (factor*np.asarray(contact["force_on_body2"], float)).tolist()
            extra.append(dict(body=bar, point=row["point"], force=((1.0-factor)*force_bar).tolist()))
        if extra:
            frame.setdefault("external", []).extend(extra)
    _restore_system_balance(out, bar)
    return out


def move_load_to_segment(frames, segment="forearm", touch_fraction=.05, bar=BAR_BODY):
    """Relabel the load-bearing permitted contacts onto a non-grasp segment body.

    Every permitted pad keeps a small ``touch_fraction`` of its original force
    at the same point (fingers still touching); the relabelled segment contact
    at that point carries the rest. The bar-side wrench and the whole-system
    balance stay exact, but only the small finger touches count as grasp, so
    the hand support ratio is ~``touch_fraction``. Keeping the full pad set
    also keeps the load-weighted contact site as stable as the genuine hold,
    so the scene exercises the support gate rather than a contact-site slip.
    """
    out = copy.deepcopy(frames)
    body_l, body_r, geom_l, geom_r = SEGMENTS[segment]
    for frame in out:
        ledger = classify_frame(frame, bar)
        touches = []
        for row in ledger["contacts"]:
            contact = frame["contacts"][row["index"]]
            original_force = np.asarray(contact["force_on_body2"], float)
            touch = dict(contact)
            touch["body1" if contact["body2"] == bar else "body2"] = row["body"]
            touch["geom1" if contact["body2"] == bar else "geom2"] = row["geom"]
            touch["force_on_body2"] = (touch_fraction*original_force).tolist()
            touches.append(touch)
            contact["force_on_body2"] = ((1.-touch_fraction)*original_force).tolist()
            if contact["body2"] == bar:
                contact["body1"] = body_l if row["side"] == "l" else body_r
                contact["geom1"] = geom_l if row["side"] == "l" else geom_r
            else:
                contact["body2"] = body_l if row["side"] == "l" else body_r
                contact["geom2"] = geom_l if row["side"] == "l" else geom_r
        frame["contacts"].extend(touches)
    return out


def forge_grasp_geom_body(frames, index=-1, bar=BAR_BODY):
    """Rename a permitted pad geom but leave it on a non-grasp body."""
    out = copy.deepcopy(frames)
    frame = out[index]
    ledger = classify_frame(frame, bar)
    row = ledger["contacts"][0]
    contact = frame["contacts"][row["index"]]
    other = "body1" if contact["body2"] == bar else "body2"
    contact[other] = "l_forearm"
    return out


def forge_contact_point(frames, offset=(0.0, 0.0, 0.30), index=-1, bar=BAR_BODY):
    """Keep a registered pad identity but move its contact point off the pad body."""
    out = copy.deepcopy(frames)
    frame = out[index]
    ledger = classify_frame(frame, bar)
    row = ledger["contacts"][0]
    contact = frame["contacts"][row["index"]]
    contact["point"] = (np.asarray(contact["point"], float)+np.asarray(offset, float)).tolist()
    return out


def swap_contact_sites(frames, bar=BAR_BODY):
    """Assign the thumb forces to the other thumb contact site: same net force, wrong moment."""
    out = copy.deepcopy(frames)
    for frame in out:
        ledger = classify_frame(frame, bar)
        for side in ("l", "r"):
            thumb = [r for r in ledger["contacts"] if r["side"] == side and r["role"] == "THUMB"]
            if len(thumb) < 2:
                continue
            a, b = thumb[0]["index"], thumb[1]["index"]
            ca, cb = frame["contacts"][a], frame["contacts"][b]
            ca["force_on_body2"], cb["force_on_body2"] = cb["force_on_body2"], ca["force_on_body2"]
    return out


def reverse_one_hand(frames, side="l", bar=BAR_BODY):
    """Reverse the sign of one hand's contact forces (adhesive contact)."""
    out = copy.deepcopy(frames)
    for frame in out:
        ledger = classify_frame(frame, bar)
        for row in ledger["contacts"]:
            if row["side"] == side:
                contact = frame["contacts"][row["index"]]
                contact["force_on_body2"] = (-np.asarray(contact["force_on_body2"], float)).tolist()
    return out


def scale_bar_inertia(frames, factor=1.2, bar=BAR_BODY):
    """Scale the exported barbell inertia without changing its mass or trajectory."""
    out = copy.deepcopy(frames)
    for frame in out:
        body = next(b for b in frame["bodies"] if b["name"] == bar)
        body["inertia_world"] = (factor*np.asarray(body["inertia_world"], float)).tolist()
    return out


def segment_only(frames, segment="thigh", bar=BAR_BODY):
    """Relabel every permitted hand contact onto a segment: no permitted grasp contact remains."""
    out = copy.deepcopy(frames)
    body_l, body_r, geom_l, geom_r = SEGMENTS[segment]
    for frame in out:
        ledger = classify_frame(frame, bar)
        for row in ledger["contacts"]:
            contact = frame["contacts"][row["index"]]
            if contact["body2"] == bar:
                contact["body1"] = body_l if row["side"] == "l" else body_r
                contact["geom1"] = geom_l if row["side"] == "l" else geom_r
            else:
                contact["body2"] = body_l if row["side"] == "l" else body_r
                contact["geom2"] = geom_l if row["side"] == "l" else geom_r
    return out

def rotate_reported_bar(frames, tilt_deg=0.0, twist_deg=0.0, bar=BAR_BODY):
    """Rotate only the *reported* bar orientation.

    ``tilt_deg`` rotates the shaft axis away from the hand grip axis (the
    invalid "large rotation with almost no COM translation" control);
    ``twist_deg`` rolls the bar about its own shaft (a permitted rotation for
    the declared axisymmetric unplated shaft).
    """
    out = copy.deepcopy(frames)
    for frame in out:
        R = np.asarray(frame["bar_frame"]["rotation"], float)
        if tilt_deg:
            a = math.radians(tilt_deg)
            R = np.array([[1., 0., 0.], [0., math.cos(a), -math.sin(a)], [0., math.sin(a), math.cos(a)]]) @ R
        if twist_deg:
            a = math.radians(twist_deg)
            R = R @ np.array([[math.cos(a), 0., math.sin(a)], [0., 1., 0.], [-math.sin(a), 0., math.cos(a)]])
        frame["bar_frame"]["rotation"] = R.tolist()
        frame["bar_frame"]["axis"] = R[:, 1].tolist()
    return out


def translate_bar_axially(frames, shift_m=.005, start_s=.1, bar=BAR_BODY):
    """Slide the bar along its own shaft axis relative to the hands.

    The bar's COM, body-frame origin and the bar-side contact points all move by
    the same amount, so the contact wrench and the angular balance about the bar
    COM are unchanged; only the hand-relative axial position changes. That
    isolates the axial-slip criterion from the balance checks.
    """
    out = copy.deepcopy(frames)
    t0, t1 = float(start_s), float(frames[-1]["t"])
    for frame in out:
        frac = 0.0 if t1 <= t0 else max(0., min(1., (frame["t"]-t0)/(t1-t0)))
        d = shift_m*frac*np.asarray(frame["bar_frame"]["axis"], float)
        frame["bar_frame"]["position"] = (np.asarray(frame["bar_frame"]["position"], float)+d).tolist()
        body = next(b for b in frame["bodies"] if b["name"] == bar)
        body["com"] = (np.asarray(body["com"], float)+d).tolist()
        for c in frame["contacts"]:
            if bar in (c["body1"], c["body2"]):
                c["point"] = (np.asarray(c["point"], float)+d).tolist()
    return out


def rotate_reported_bar_ramp(frames, tilt_deg=0.0, twist_deg=0.0, start_s=.1, bar=BAR_BODY):
    """Rotate only the reported bar orientation progressively after settling."""
    out = copy.deepcopy(frames)
    t0, t1 = float(start_s), float(frames[-1]["t"])
    for frame in out:
        frac = 0.0 if t1 <= t0 else max(0., min(1., (frame["t"]-t0)/(t1-t0)))
        R = np.asarray(frame["bar_frame"]["rotation"], float)
        if tilt_deg and frac:
            a = math.radians(tilt_deg*frac)
            R = np.array([[1., 0., 0.], [0., math.cos(a), -math.sin(a)], [0., math.sin(a), math.cos(a)]]) @ R
        if twist_deg and frac:
            a = math.radians(twist_deg*frac)
            R = R @ np.array([[math.cos(a), 0., math.sin(a)], [0., 1., 0.], [-math.sin(a), 0., math.cos(a)]])
        frame["bar_frame"]["rotation"] = R.tolist()
        frame["bar_frame"]["axis"] = R[:, 1].tolist()
    return out


def offset_hand_frame(frames, side="r", angle_deg=10.0, axis="x"):
    """Report a constant hand-frame orientation offset (wrong grip frame)."""
    out = copy.deepcopy(frames)
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    Roff = {"x": np.array([[1., 0., 0.], [0., c, -s], [0., s, c]]),
            "y": np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]]),
            "z": np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])}[axis]
    for frame in out:
        Rh = np.asarray(frame["hand_frames"][side]["rotation"], float)
        frame["hand_frames"][side]["rotation"] = (Roff@Rh).tolist()
    return out


def ramp_hand_frames(frames, side="r", angle_deg=5.0, axis="x"):
    """Rotate one reported hand frame progressively (hands rotating in the grip)."""
    out = copy.deepcopy(frames)
    t0, t1 = out[0]["t"], out[-1]["t"]
    for frame in out:
        frac = 0.0 if t1 <= t0 else max(0., min(1., (frame["t"]-t0)/(t1-t0)))
        a = math.radians(angle_deg*frac)
        c, s = math.cos(a), math.sin(a)
        Roff = {"x": np.array([[1., 0., 0.], [0., c, -s], [0., s, c]]),
                "y": np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]]),
                "z": np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])}[axis]
        Rh = np.asarray(frame["hand_frames"][side]["rotation"], float)
        frame["hand_frames"][side]["rotation"] = (Roff@Rh).tolist()
    return out


def small_legitimate_relative_motion(frames, side="r", tilt_deg=.5, hand_shift_m=.0005, bar=BAR_BODY):
    """Small orientation/position change inside the declared grip bounds (must pass)."""
    out = rotate_reported_bar(frames, tilt_deg=tilt_deg, bar=bar)
    d = np.array([hand_shift_m, 0., 0.])
    for frame in out:
        frame["hand_frames"][side]["position"] = (np.asarray(frame["hand_frames"][side]["position"], float)+d).tolist()
    return out
