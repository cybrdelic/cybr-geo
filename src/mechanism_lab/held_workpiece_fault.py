"""Pure, source-bound motor-off contract for held free workpieces.

The release caller supplies an externally controlled acceptance profile and trust
policy, then passes only measurements from its verified signed evidence set.
This module does not authenticate reports or establish that a test took place.
"""
from __future__ import annotations

from copy import deepcopy
import math
import re
from xml.etree.ElementTree import Element


class HeldWorkpieceFaultError(ValueError):
    """A motor-off source, case, or observation is absent, invalid, or stale."""


_SHA = re.compile(r"[0-9a-f]{64}\Z")
_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")
_CASE_FIELDS = {"workpiece_body", "contact_pairs", "actuators_off",
                "workpiece_mass_kg", "maximum_mass_deviation_kg",
                "gravity_vector_m_s2",
                "minimum_observation_s", "maximum_downward_drop_m",
                "maximum_residual_current_A", "maximum_residual_torque_Nm"}
_OBS_FIELDS = {"case_id", "source_sha256", "model_sha256", "workpiece_body",
               "assembled_serial", "part_serials", "workpiece_mass_kg", "power_removed",
               "actuators_off", "gravity_vector_m_s2", "initial_contact_pairs",
               "initially_suspended_only_by_holder_contacts",
               "other_supports_absent", "commanded_current_A", "commanded_torque_Nm",
               "maximum_measured_current_A", "maximum_measured_torque_Nm",
               "observation_s", "maximum_downward_drop_m"}
_SIM_FIELDS = (_OBS_FIELDS - {"assembled_serial", "part_serials"}) | {"trace_sha256"}


def _object(value, label):
    if not isinstance(value, dict):
        raise HeldWorkpieceFaultError(f"{label} must be an object")
    return value


def _fields(value, fields, label):
    if set(_object(value, label)) != fields:
        raise HeldWorkpieceFaultError(f"{label} fields must be exactly {sorted(fields)}")


def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise HeldWorkpieceFaultError(f"{label} must be nonempty text")
    return value


def _sha(value, label):
    if not isinstance(value, str) or not _SHA.fullmatch(value):
        raise HeldWorkpieceFaultError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _number(value, label, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (positive and value == 0)):
        raise HeldWorkpieceFaultError(f"{label} must be finite and {'positive' if positive else 'nonnegative'}")
    return float(value)


def _names(value, label):
    if (not isinstance(value, list) or not value
            or any(not isinstance(name, str) or not name.strip() for name in value)
            or value != sorted(value) or len(set(value)) != len(value)):
        raise HeldWorkpieceFaultError(f"{label} must be sorted unique names")
    return value


def _pairs(value, label):
    if (not isinstance(value, list) or not value
            or any(not isinstance(pair, list) or len(pair) != 2
                   or any(not isinstance(name, str) or not name.strip() for name in pair)
                   for pair in value)
            or value != sorted(value)
            or len({tuple(pair) for pair in value}) != len(value)):
        raise HeldWorkpieceFaultError(f"{label} must be sorted unique [moving_geom, free_geom] pairs")
    return value


def _gravity(root):
    option = root.find("option")
    if option is None or not option.get("gravity"):
        raise HeldWorkpieceFaultError("Motor-off retention requires explicit source gravity")
    try:
        gravity = [float(value) for value in option.get("gravity").split()]
    except ValueError as error:
        raise HeldWorkpieceFaultError("Source gravity is invalid") from error
    if len(gravity) != 3 or any(not math.isfinite(value) for value in gravity) or not any(gravity):
        raise HeldWorkpieceFaultError("Source gravity must be a finite nonzero 3-vector")
    return gravity


def _gravity_row(value, label):
    if (not isinstance(value, list) or len(value) != 3
            or any(isinstance(item, bool) or not isinstance(item, (int, float))
                   or not math.isfinite(item) for item in value)):
        raise HeldWorkpieceFaultError(f"{label} must be a finite gravity 3-vector")
    return [float(item) for item in value]


def held_free_workpieces(root: Element) -> dict[str, dict]:
    """Find freejoint bodies with explicit contact to at least one moving body.

    This conservatively flags pad candidates, not proof of gripping or retention.
    Even one moving contact triggers the gate so deleting a second pad pair cannot
    silently bypass it. An unclassified free body is an error for release.
    """
    if not isinstance(root, Element) or root.tag != "mujoco":
        raise HeldWorkpieceFaultError("A parsed MuJoCo root is required")
    world = root.find("worldbody")
    if world is None:
        raise HeldWorkpieceFaultError("MuJoCo worldbody is missing")
    geoms = {}  # geom name -> (owning body, moves, freejoint ancestor)
    free_bodies = {}
    bodies = set()

    def visit(body, moving=False, free_owner=None):
        name = _text(body.get("name"), "MuJoCo body name")
        if name in bodies:
            raise HeldWorkpieceFaultError(f"Duplicate MuJoCo body {name}")
        bodies.add(name)
        joints, free = body.findall("joint"), body.findall("freejoint")
        if len(free) > 1 or (free and joints) or (free and free_owner):
            raise HeldWorkpieceFaultError(f"Ambiguous freejoint on body {name}")
        if free:
            inertial = body.find("inertial")
            try:
                mass = float(inertial.get("mass", "")) if inertial is not None else None
            except ValueError as error:
                raise HeldWorkpieceFaultError(f"Freejoint body {name} has invalid mass") from error
            free_bodies[name] = _number(mass, f"Freejoint body {name} mass", positive=True)
            free_owner = name
        moving = moving or bool(joints)
        for geom in body.findall("geom"):
            geom_name = geom.get("name")
            if geom_name:
                if geom_name in geoms:
                    raise HeldWorkpieceFaultError(f"Duplicate MuJoCo geom {geom_name}")
                geoms[geom_name] = (name, moving, free_owner)
        for child in body.findall("body"):
            visit(child, moving, free_owner)

    for body in world.findall("body"):
        visit(body)
    actuator_root = root.find("actuator")
    actuators = [] if actuator_root is None else [item.get("name") for item in actuator_root]
    if free_bodies and (not actuators or any(not name for name in actuators)
                        or len(set(actuators)) != len(actuators)):
        raise HeldWorkpieceFaultError("Freejoint retention needs a complete named actuator inventory")
    contacts = {name: [] for name in free_bodies}
    holders = {name: set() for name in free_bodies}
    for pair in root.findall("./contact/pair"):
        first, second = pair.get("geom1"), pair.get("geom2")
        if first not in geoms or second not in geoms:
            raise HeldWorkpieceFaultError("Explicit contact pair names an absent or unnamed geom")
        for free_geom, moving_geom in ((first, second), (second, first)):
            owner = geoms[free_geom][2]
            if owner in free_bodies and geoms[moving_geom][1] and geoms[moving_geom][2] != owner:
                contacts[owner].append([moving_geom, free_geom])
                holders[owner].add(geoms[moving_geom][0])
    found = {}
    for name, mass in free_bodies.items():
        if not contacts[name]:
            raise HeldWorkpieceFaultError(f"Freejoint body {name} has no explicit moving-holder contact")
        if len({tuple(pair) for pair in contacts[name]}) != len(contacts[name]):
            raise HeldWorkpieceFaultError(f"Freejoint body {name} has duplicate contact pairs")
        found[name] = dict(workpiece_mass_kg=mass, contact_pairs=sorted(contacts[name]),
                           holder_bodies=sorted(holders[name]), actuators_off=sorted(actuators))
    return found


def require_motor_off_fault_cases(root: Element, *, source_sha256: str,
                                  model_sha256: str, external_profile: dict | None,
                                  trust_policy: dict | None,
                                  planned_cases: dict | None) -> dict:
    """Match exact externally required cases to source geometry, mass, and drives.

    Caller must enforce external file locations. Its v1 signature verifier must
    separately establish authenticity of later ``fault_test`` observations.
    """
    _sha(source_sha256, "Source revision")
    _sha(model_sha256, "MuJoCo model")
    held = held_free_workpieces(root)
    if not held:
        if external_profile is not None or planned_cases:
            raise HeldWorkpieceFaultError("Motor-off cases declare no held free workpiece")
        return dict(schema="cybrgeo-held-workpiece-fault-requirements/1",
                    source_sha256=source_sha256, model_sha256=model_sha256, cases={})
    gravity = _gravity(root)
    _fields(external_profile, {"schema", "source_sha256", "model_sha256", "cases"},
            "External motor-off acceptance profile")
    if external_profile["schema"] != "cybrgeo-held-workpiece-fault-profile/1":
        raise HeldWorkpieceFaultError("Unsupported motor-off acceptance profile schema")
    if (_sha(external_profile["source_sha256"], "Profile source revision") != source_sha256
            or _sha(external_profile["model_sha256"], "Profile model") != model_sha256):
        raise HeldWorkpieceFaultError("Motor-off acceptance profile is stale for source/model")
    policy = _object(trust_policy, "External trust policy")
    if policy.get("schema") != "cybrgeo-trust-policy/1":
        raise HeldWorkpieceFaultError("Expected external cybrgeo-trust-policy/1")
    issuers = _object(policy.get("issuers"), "Trusted issuers")
    if not any(isinstance(issuer, dict) and isinstance(issuer.get("allowed_kinds"), list)
               and "fault_test" in issuer["allowed_kinds"]
               and isinstance(issuer.get("public_key_base64"), str)
               and issuer["public_key_base64"] for issuer in issuers.values()):
        raise HeldWorkpieceFaultError("External trust policy lacks a fault_test issuer")
    cases = _object(external_profile["cases"], "Required motor-off cases")
    plan = _object(planned_cases, "Source-planned motor-off cases")
    if not cases or set(plan) != set(cases):
        raise HeldWorkpieceFaultError("Source plan must name exactly the external motor-off case IDs")
    covered = set()
    for case_id, row in cases.items():
        if not isinstance(case_id, str) or not _ID.fullmatch(case_id):
            raise HeldWorkpieceFaultError("Motor-off case ID must be portable and nonempty")
        _fields(row, _CASE_FIELDS, f"Motor-off case {case_id}")
        body = _text(row["workpiece_body"], f"{case_id} workpiece body")
        if body not in held:
            raise HeldWorkpieceFaultError(f"{case_id} names no held free workpiece")
        actual = held[body]
        if _pairs(row["contact_pairs"], f"{case_id} contacts") != actual["contact_pairs"]:
            raise HeldWorkpieceFaultError(f"{case_id} contacts differ from source moving pads")
        if _names(row["actuators_off"], f"{case_id} actuators") != actual["actuators_off"]:
            raise HeldWorkpieceFaultError(f"{case_id} does not switch off every source actuator")
        if (_number(row["workpiece_mass_kg"], f"{case_id} workpiece mass", positive=True)
                != actual["workpiece_mass_kg"]):
            raise HeldWorkpieceFaultError(f"{case_id} workpiece mass differs from source")
        if _gravity_row(row["gravity_vector_m_s2"], f"{case_id} gravity") != gravity:
            raise HeldWorkpieceFaultError(f"{case_id} gravity differs from source")
        for key in ("maximum_mass_deviation_kg", "maximum_downward_drop_m",
                    "maximum_residual_current_A", "maximum_residual_torque_Nm"):
            _number(row[key], f"{case_id} {key}")
        _number(row["minimum_observation_s"], f"{case_id} minimum duration", positive=True)
        if plan[case_id] != row:
            raise HeldWorkpieceFaultError(f"{case_id} source plan differs from external acceptance case")
        covered.add(body)
    if covered != set(held):
        raise HeldWorkpieceFaultError("Motor-off cases do not cover every held free workpiece")
    return dict(schema="cybrgeo-held-workpiece-fault-requirements/1",
                source_sha256=source_sha256, model_sha256=model_sha256,
                cases=deepcopy(cases))


def _current_cases(requirements, source_sha256, model_sha256):
    _fields(requirements, {"schema", "source_sha256", "model_sha256", "cases"},
            "Motor-off requirements")
    if requirements["schema"] != "cybrgeo-held-workpiece-fault-requirements/1":
        raise HeldWorkpieceFaultError("Unsupported motor-off requirements schema")
    if (_sha(source_sha256, "Source revision") != requirements["source_sha256"]
            or _sha(model_sha256, "MuJoCo model") != requirements["model_sha256"]):
        raise HeldWorkpieceFaultError("Motor-off requirements are stale for source/model")
    return _object(requirements["cases"], "Required motor-off cases")


def _check_dynamic_row(case_id, row, case, source_sha256, model_sha256):
    if (row["case_id"] != case_id or row["source_sha256"] != source_sha256
            or row["model_sha256"] != model_sha256
            or row["workpiece_body"] != case["workpiece_body"]):
        raise HeldWorkpieceFaultError(f"{case_id} observation is bound to another case/source/model")
    mass = _number(row["workpiece_mass_kg"], f"{case_id} measured workpiece mass", positive=True)
    if abs(mass - case["workpiece_mass_kg"]) > case["maximum_mass_deviation_kg"]:
        raise HeldWorkpieceFaultError(f"{case_id} tested workpiece mass is outside source-bound range")
    if row["power_removed"] is not True or row["actuators_off"] != case["actuators_off"]:
        raise HeldWorkpieceFaultError(f"{case_id} observation did not remove power from all actuators")
    if (_gravity_row(row["gravity_vector_m_s2"], f"{case_id} observed gravity")
            != case["gravity_vector_m_s2"]
            or row["initial_contact_pairs"] != case["contact_pairs"]
            or row["initially_suspended_only_by_holder_contacts"] is not True
            or row["other_supports_absent"] is not True):
        raise HeldWorkpieceFaultError(
            f"{case_id} was not observed initially suspended only by source holder contacts")
    for key in ("commanded_current_A", "commanded_torque_Nm"):
        if _number(row[key], f"{case_id} {key}") != 0:
            raise HeldWorkpieceFaultError(f"{case_id} has nonzero motor-off command")
    for observed, limit in (("maximum_measured_current_A", "maximum_residual_current_A"),
                            ("maximum_measured_torque_Nm", "maximum_residual_torque_Nm")):
        if _number(row[observed], f"{case_id} {observed}") > case[limit]:
            raise HeldWorkpieceFaultError(f"{case_id} exceeds source-bound {limit}")
    if (_number(row["observation_s"], f"{case_id} observed duration")
            < case["minimum_observation_s"]):
        raise HeldWorkpieceFaultError(f"{case_id} observation duration is too short")
    if (_number(row["maximum_downward_drop_m"], f"{case_id} measured drop")
            > case["maximum_downward_drop_m"]):
        raise HeldWorkpieceFaultError(f"{case_id} workpiece drop exceeds source-bound limit")


def validate_motor_off_fault_simulations(requirements: dict, *, source_sha256: str,
                                         model_sha256: str, results: dict | None) -> dict:
    """Validate source-bound dynamics receipt fields, not the trace or solver.

    Release must verify trace bytes/hash and that the source model actually ran
    with the prescribed fault; a self-reported receipt alone proves neither.
    """
    cases = _current_cases(requirements, source_sha256, model_sha256)
    rows = _object(results, "Motor-off simulation receipts")
    if set(rows) != set(cases):
        raise HeldWorkpieceFaultError("Motor-off simulation receipts must cover exact required case IDs")
    for case_id, row in rows.items():
        _fields(row, _SIM_FIELDS, f"Motor-off simulation {case_id}")
        _sha(row["trace_sha256"], f"{case_id} simulation trace")
        _check_dynamic_row(case_id, row, cases[case_id], source_sha256, model_sha256)
    return deepcopy(rows)


def validate_motor_off_fault_observations(
        requirements: dict, *, source_sha256: str, model_sha256: str,
        assembled_serial: str, part_serials: dict, observations: dict | None) -> dict:
    """Check fields of already signature-verified same-unit observations.

    This cannot verify signatures, instrument calibration, time-series traces,
    or physical test authenticity. The release caller must do so separately.
    """
    cases = _current_cases(requirements, source_sha256, model_sha256)
    serial = _text(assembled_serial, "Assembled unit serial")
    bom = _object(part_serials, "Inspected assembled-unit part serials")
    if not bom or any(not isinstance(name, str) or not name or not isinstance(value, str)
                      or not value for name, value in bom.items()):
        raise HeldWorkpieceFaultError("Inspected assembled-unit part serials are incomplete")
    rows = _object(observations, "Signed motor-off observations")
    if set(rows) != set(cases):
        raise HeldWorkpieceFaultError("Signed motor-off observations must cover exact required case IDs")
    for case_id, row in rows.items():
        _fields(row, _OBS_FIELDS, f"Motor-off observation {case_id}")
        if row["assembled_serial"] != serial or row["part_serials"] != bom:
            raise HeldWorkpieceFaultError(f"{case_id} observation is not for the inspected assembled unit")
        _check_dynamic_row(case_id, row, cases[case_id], source_sha256, model_sha256)
    return deepcopy(rows)
