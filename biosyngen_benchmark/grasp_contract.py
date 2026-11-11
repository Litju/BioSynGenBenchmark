"""RES-624 Gate A: versioned independent grasp-contact contract.

The receiving evaluator may not import generator internals, so the permitted
grasping bodies, the contact geometries compiled onto them and the physical
interface classes are declared here explicitly. ``tests/test_res624_grasp_contract``
binds this contract to the compiled Human V2 model, so generator drift is a
test failure rather than a silent acceptance; the evaluator itself consumes
only this registry.

Only four interfaces per hand may transmit grip load:

===============  =======================  =====================
geom             body                     interface
===============  =======================  =====================
*_palm_collision  *_hand                  PALM  (palmar surface)
*_finger_flex_pad *_fingers_proximal      FINGER_PROXIMAL
*_finger_distal_pad *_fingers_distal      FINGER_DISTAL
*_thumb_opposition_pad *_thumb           THUMB
===============  =======================  =====================

Forearm, upper arm, thigh, shank, foot, torso and head are explicitly *not*
grasp interfaces even though their body names share the ``l_``/``r_`` prefix
that the pre-repair evaluator treated as hand contact. A contact is a grasp
contact only if the barbell body, the bar geom, the human body, the human
geom and the contact point all match the declared interface.

NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CONTRACT_VERSION = "RES624_GRASP_CONTACT_CONTRACT_V1"
BAR_BODY = "barbell"
BAR_GRASP_GEOM = "bar_shaft"
SIDES = ("l", "r")
# The compiled pad/shaft friction is 1.0. A larger declared value could hide a
# friction-cone violation, so the evaluator refuses it instead of trusting it.
MAX_DECLARED_FRICTION = 1.0 + 1e-9
# A contact transmitting less than this fraction of the bar weight is geometric
# proximity, not load-bearing contact (0.098 N for the 20 kg bar).
ACTIVE_NORMAL_FRACTION = 5e-4
# Contact-point binding to the registered body COM. Measured maxima on the
# nominal hold are 0.0354 m (finger pads) and 0.0150 m (thumb pad); the values
# below add margin for stature scaling and the box-contact manifold while
# remaining far below the distance from a hand to any other segment.
MAX_POINT_OFFSET_M = dict(PALM=.10, FINGER_PROXIMAL=.06, FINGER_DISTAL=.06, THUMB=.05)
# Bilateral acceptance: each hand must carry at least this share of the bar's
# weight impulse. 2 % (3.9 N for the 20 kg bar) is 40x the active-contact
# threshold and far below any deliberate asymmetric two-hand sharing, so a
# genuine 30/70 or 5/95 split still qualifies while a resting hand does not.
BILATERAL_MIN_SHARE = .02


@dataclass(frozen=True)
class GraspInterface:
    geom: str
    body: str
    side: str
    role: str


GRASP_INTERFACES = tuple(GraspInterface(f"{s}_{g}", f"{s}_{b}", s, role)
                         for s in SIDES
                         for g, b, role in (("palm_collision", "hand", "PALM"),
                                            ("finger_flex_pad", "fingers_proximal", "FINGER_PROXIMAL"),
                                            ("finger_distal_pad", "fingers_distal", "FINGER_DISTAL"),
                                            ("thumb_opposition_pad", "thumb", "THUMB")))
GRASP_GEOMS = {i.geom: i for i in GRASP_INTERFACES}
GRASP_BODIES = {i.body: i for i in GRASP_INTERFACES}
if len(GRASP_GEOMS) != 8 or len(GRASP_BODIES) != 8:  # pragma: no cover - construction guard
    raise AssertionError("grasp contract must declare four interfaces per side")


def _finite(*values):
    return all(math.isfinite(v) for v in values)


def _bar_force(contact, bar):
    """World force on the bar from one contact, and its point."""
    f = np.asarray(contact["force_on_body2"], float)
    return (f if contact["body2"] == bar else -f), np.asarray(contact["point"], float)


def _row(contact, bar, interface, bodies, active_n):
    bar_geom = contact["geom2"] if contact["body2"] == bar else contact["geom1"]
    force, point = _bar_force(contact, bar)
    normal = np.asarray(contact["normal"], float)
    mu = float(contact["friction"])
    f2 = np.asarray(contact["force_on_body2"], float)
    fn = float(f2 @ normal) if _finite(*normal, *f2) and abs(float(np.linalg.norm(normal))-1) <= 1e-9 else float("nan")
    ft = float(np.linalg.norm(f2 - fn*normal)) if math.isfinite(fn) else float("nan")
    problems = []
    if not _finite(*normal, *f2) or abs(float(np.linalg.norm(normal))-1) > 1e-9:
        problems.append("NONFINITE_OR_NONUNIT_NORMAL")
    elif fn < -1e-9*max(1.0, float(np.linalg.norm(f2))):
        problems.append("ADHESIVE_NORMAL_FORCE")
    elif ft > mu*max(fn, 0.)*(1+1e-6)+1e-9:
        problems.append("OUTSIDE_FRICTION_CONE")
    if mu > MAX_DECLARED_FRICTION:
        problems.append("UNDECLARED_FRICTION")
    body_com = np.asarray(bodies[interface.body]["com"], float)
    offset = float(np.linalg.norm(point-body_com))
    identity = []
    if bar_geom != BAR_GRASP_GEOM:
        identity.append("BAR_GRASP_GEOM_MISMATCH")
    if offset > MAX_POINT_OFFSET_M[interface.role]:
        identity.append("CONTACT_POINT_OFF_REGISTERED_INTERFACE")
    return dict(index=contact.get("index"), side=interface.side, role=interface.role, geom=interface.geom,
                body=interface.body, bar_geom=bar_geom, point=point.tolist(), force_on_bar=force.tolist(),
                normal=normal.tolist(), friction=mu, normal_force=fn, tangential_force=ft,
                point_offset_m=offset, active=bool(math.isfinite(fn) and fn >= active_n),
                admissible=not problems, problems=problems, identity_ok=not identity, identity_problems=identity)


def classify_frame(frame, bar=BAR_BODY):
    """Permitted grasp contacts, identity problems and non-grasp bar contacts of one frame.

    The ledger is deliberately conservative: a contact is a *geometric* grasp
    contact only when its bar geom, human body, human geom and point are all
    within the declared interface. Contacts on any other body (forearm, thigh,
    shin, torso, head, ...) are reported as non-grasp bar contacts and never
    contribute support. Contacts that claim a registered geom on the wrong
    body, or a point far from the registered body, are identity problems.
    """
    bodies = {b["name"]: b for b in frame["bodies"]}
    if bar not in bodies:
        return dict(contract=CONTRACT_VERSION, contacts=[], identity_problems=[("BAR_BODY_MISSING", bar)],
                    nongrasp=[], bar_contacts=0)
    weight = float(bodies[bar]["mass"])*float(np.linalg.norm(frame["gravity"]))
    active_n = ACTIVE_NORMAL_FRACTION*weight
    out = dict(contract=CONTRACT_VERSION, contacts=[], identity_problems=[], nongrasp=[], bar_contacts=0)
    for index, contact in enumerate(frame["contacts"]):
        if bar not in (contact["body1"], contact["body2"]):
            continue
        out["bar_contacts"] += 1
        contact = dict(contact, index=index)
        other_body = contact["body1"] if contact["body2"] == bar else contact["body2"]
        other_geom = contact["geom1"] if contact["body2"] == bar else contact["geom2"]
        interface = GRASP_GEOMS.get(other_geom)
        if interface is None:
            if other_body in GRASP_BODIES:
                out["identity_problems"].append(("UNREGISTERED_GEOM_ON_GRASP_BODY", other_body, other_geom))
            out["nongrasp"].append(dict(body=other_body, geom=other_geom,
                                        force_on_bar=_bar_force(contact, bar)[0].tolist(),
                                        point=list(contact["point"])))
            continue
        if interface.body != other_body:
            out["identity_problems"].append(("GRASP_BODY_GEOM_MISMATCH", other_geom, other_body, interface.body))
            continue
        row = _row(contact, bar, interface, bodies, active_n)
        out["identity_problems"].extend((p, row["geom"], row["body"]) for p in row["identity_problems"])
        if row["identity_ok"]:
            out["contacts"].append(row)
    return out


def interface_table():
    """Plain audit table of the versioned registry (used by the binding test and receipts)."""
    return dict(contract=CONTRACT_VERSION, bar_body=BAR_BODY, bar_grasp_geom=BAR_GRASP_GEOM,
                max_declared_friction=MAX_DECLARED_FRICTION, active_normal_fraction=ACTIVE_NORMAL_FRACTION,
                bilateral_min_share=BILATERAL_MIN_SHARE, max_point_offset_m=dict(MAX_POINT_OFFSET_M),
                interfaces=[dict(geom=i.geom, body=i.body, side=i.side, role=i.role) for i in GRASP_INTERFACES])
