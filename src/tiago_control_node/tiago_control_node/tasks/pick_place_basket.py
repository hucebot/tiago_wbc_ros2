"""Task: pick the object up off the table and drop it into the basket fixture.

Everything in this file is plain waypoint/geometry data and a success check - no ROS2, no
OpenSoT. This is the file to read/edit if you want to change *what* the robot does; the
engine that actually runs a PLAN (pose_commander.py's PoseCommander) doesn't know or care
what's in it.

`PLAN` is a sequence of waypoints for pose_commander.PoseCommander.run_plan(), which splines
ALL of them into one continuous motion (see its docstring) rather than running each in
isolation - 'hold' is how many seconds after the PREVIOUS waypoint this one is reached, not
a pause after arriving. A waypoint with no 'right'/'left' key (e.g. a gripper-only step)
doesn't move that side at all - the spline just holds its position flat across that time
span. Orientation can be given as 'rpy_deg' (roll, pitch, yaw in degrees) or 'quat' (x, y, z,
w). A 'gripper' key commands the MuJoCo bridge directly, e.g. {'right': 'close'}.

Position is given as 'xyz_rel', an [dx, dy, dz] offset added to the target object's xyz
(captured once, at plan start) - or as 'xyz', an absolute base_link-frame position, for
waypoints that aren't object-relative (e.g. the basket: it's a fixed static fixture, not the
tracked object, so its waypoint uses its known world position directly instead of an offset).

Pick-and-place-in-basket demo: right arm picks the object up off the table and drops it into
the basket fixture (robots/pal_tiago_pro/xmls/scene_tiago_pro.xml, body "basket" -
BASKET_XY/BASKET_HOVER_Z below must match that body's geometry). quat [1,0,0,0] is a top-down
approach (gripper pointing straight down).

APPROACH_Z_OFFSET/GRASP_Z_OFFSET (pick side) are tuned/verified-reachable against the
robots/pal_tiago_pro MuJoCo model - pose_commander targets the real solver's
gripper_right_grasping_link frame from the actual URDF, whose exact offset from the wrist may
not perfectly match the MuJoCo model, so nudge by a centimeter or two if the real grasp
misses. The grasp orientation ignores the object's own rotation (fine for a symmetric cube;
revisit if the object changes).

Table strikes were happening because pregrasp/descend/lift each used a different xy (and the
very first move went straight from home - which isn't top-down - to a low, angled pregrasp),
so the arm was simultaneously reorienting, translating laterally, AND dropping close to the
table in one motion, with no guarantee OpenSoT's IK path stays clear of it along the way.
Fixed by decoupling those three things: GRASP_XY_OFFSET is now shared by every pick-phase
waypoint (pregrasp, descend, lift, transit) so the only thing that ever changes near the
table is height, one axis at a time; TRANSIT_Z_OFFSET adds a safe waypoint well above the
table where the big home -> top-down reorientation and any lateral travel happen, clear of
any collision risk.

The basket has 5cm walls with the floor's top at 0.48 and the rim at 0.53 (see the XML);
BASKET_HOVER_Z hovers 3cm above the rim and releases there rather than descending inside the
walls, to keep the gripper clear of them - the object free-falls the last few cm into the
basket. Watch the first few drops in the viewer and tighten BASKET_HOVER_Z if it's bouncing
out.
"""
from scipy.spatial.transform import Rotation as R

TOP_DOWN = [1.0, 0.0, 0.0, 0.0]
VERTICAL_TO_OBJECT = R.from_euler('y', 90, degrees=True).as_quat().tolist()
GRASP_XY_OFFSET = [0.00, 0.0]  # lateral nudge from the object's own xy to the actual grasp point -
                                # shared by every pick-phase waypoint, see note above
TRANSIT_Z_OFFSET = 0.04    # safe height above the object for reorienting/traveling laterally
APPROACH_Z_OFFSET = 0.023  # above the object: pre-grasp / lift height, close to the table
GRASP_Z_OFFSET = -0.070    # at the object: descend-to-grasp height
BASKET_XY = (0.78, 0.0)    # must match the "basket" body's pos in scene_tiago_pro.xml
BASKET_HOVER_Z = 0.56      # basket rim (0.53) + 3cm clearance
PLAN = [
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': VERTICAL_TO_OBJECT}},     # transit height above the grasp point - reorients to top-down here, clear of the table
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': VERTICAL_TO_OBJECT}},    # pre-grasp - straight down from transit, same xy/orientation
    {'hold': 2.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, GRASP_Z_OFFSET], 'quat': VERTICAL_TO_OBJECT}},       # descend to object - straight down, no lateral motion
    {'hold': 1.0, 'gripper': {'right': 'close'}},                                                     # grasp
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': VERTICAL_TO_OBJECT}},    # lift - straight up, same xy as grasp
    # {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': VERTICAL_TO_OBJECT}},     # lift further to transit height - straight up
    {'hold': 3.0, 'right': {'xyz': [*BASKET_XY, BASKET_HOVER_Z], 'quat': VERTICAL_TO_OBJECT}},                 # transport, hover over the basket - lateral move, already at a safe height
    {'hold': 2.0, 'gripper': {'right': 'open'}},                                                       # release - drops into the basket
    # {'hold': 2.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': TOP_DOWN}},     # retreat to transit height, back over the pick spot
]

SUCCESS_XY_TOLERANCE = 0.05   # meters: how close to the basket's center counts as "inside" (basket's
                               # inner cavity is roughly +-0.07m, see scene_tiago_pro.xml)
SUCCESS_Z_MAX = 0.60          # object must have settled into/near the basket, not still airborne


def check_success(object_xyz, pick_xyz) -> tuple[bool, str]:
    """object_xyz: the object's final (x, y, z) after the plan finished. pick_xyz: unused by
    this task (kept in the signature for tasks whose success depends on where the object
    started). Returns (success, a human-readable message describing why)."""
    target_x, target_y = BASKET_XY
    fx, fy, fz = object_xyz
    dist = ((fx - target_x) ** 2 + (fy - target_y) ** 2) ** 0.5
    success = dist < SUCCESS_XY_TOLERANCE and fz < SUCCESS_Z_MAX
    message = (
        f"Object ended at ({fx:.3f}, {fy:.3f}, {fz:.3f}), basket at ({target_x:.3f}, {target_y:.3f}), "
        f"dist={dist:.3f} -> {'SUCCESS' if success else 'FAILURE'}"
    )
    return success, message
