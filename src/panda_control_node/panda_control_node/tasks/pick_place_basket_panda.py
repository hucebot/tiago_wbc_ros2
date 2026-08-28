"""Task: pick the object up off the table and drop it into the basket fixture - Panda variant
of tiago_control_node's tasks/pick_place_basket.py, for the WBC ablation (see project memory
"panda_wbc_ablation_goal"). The scene (table/cube/basket, robots/panda/xmls/scene_panda.xml)
started as an exact copy of robots/pal_tiago_pro/xmls/scene_tiago_pro.xml's geometry, but the
table/basket position and height have since been changed to suit Panda's reach - it is NOT an
exact geometric copy of TIAGo's scene anymore, so treat this as a same-task-shape ablation
(pick off a table, drop in a basket), not a byte-for-byte identical physical setup. If you
change scene_panda.xml's table/basket again, update BASKET_XY/BASKET_HOVER_Z below to match.

Runs through pose_commander.PoseCommander unchanged (reused directly from
tiago_control_node, not copied - see that file's own docstring: it has no task-specific
knowledge). Single-arm: only the 'right' key is ever used, exactly like TIAGo Pro's own
pick_place_basket already only drives 'right' - panda_opensot_node.py listens on
/cartesian_interface/right/target_pose specifically for this reason.

TOP_DOWN below is NOT copied from TIAGo's quaternion constants - TIAGo's gripper frame and
Panda's ee_panda frame (see robots/panda/urdf/panda.urdf's header comment) have unrelated
orientation conventions, so a shared "top-down" constant would be wrong for one of them.
This one was derived numerically: at the standard Franka "ready" posture
(HOME_POSITIONS in panda_opensot_node.py), ee_panda's local +Z axis already points almost
exactly along world -Z (verified: dot product with [0,0,-1] ~= 0.99999996), so that home
orientation itself already IS a top-down grasp pose - TOP_DOWN below is that orientation,
not a separately-composed rotation.

GRASP_Z_OFFSET/APPROACH_Z_OFFSET/TRANSIT_Z_OFFSET below are REASONED starting points, not
empirically verified the way TIAGo's are (this file's author could not run the actual
OpenSoT+MuJoCo stack to test grasps - no compiled pyopensot/xbot2_interface available
outside the project's dev container). ee_panda is defined at the standard Franka TCP point
(between the fingertips - see the URDF comment), so unlike TIAGo's grasp-frame convention,
grasping the cube AT its own center height (offset 0) should already be close to correct -
but treat these as a first guess to check in the viewer and nudge, exactly like TIAGo's own
task-file comments already ask you to do for that robot.
"""
from scipy.spatial.transform import Rotation as R

# xyzw - see module docstring for how this was derived (numerically, from ee_panda's actual
# orientation at the standard Franka home posture - not composed from Euler angles).
TOP_DOWN = [0.7071068, 0.7071068, 0.0, 0.0]

GRASP_XY_OFFSET = [0.00, 0.0]   # lateral nudge from the object's own xy to the actual grasp point
TRANSIT_Z_OFFSET = 0.15   # safe height above the object for reorienting/traveling laterally - well
                          # clear of the basket rim (0.53) for the lateral transit leg
APPROACH_Z_OFFSET = 0.08  # above the object: pre-grasp / lift height, clear of the cube (half-size 0.02)
GRASP_Z_OFFSET = 0.00      # at the object: descend-to-grasp height - ee_panda's TCP convention already
                          # targets the fingertip contact point, so 0 offset means grasping at the
                          # cube's own center height, unlike TIAGo's frame which needed -0.07
BASKET_XY = (0.58, 0.1)   # must match scene_panda.xml's "basket" body pos - no longer the same as
                          # TIAGo's (0.78, 0.0) since the table/basket were moved 0.1m closer to
                          # the robot base for Panda's shorter reach
BASKET_HOVER_Z = 0.48     # basket rim (0.45, from basket body z=0.395 + wall top 0.055) + 3cm
                          # clearance - recompute this if scene_panda.xml's basket pos/geometry changes

PLAN = [
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': TOP_DOWN}},   # transit height above the grasp point
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': TOP_DOWN}},  # pre-grasp - straight down from transit
    {'hold': 2.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, GRASP_Z_OFFSET], 'quat': TOP_DOWN}},      # descend to object
    {'hold': 1.0, 'gripper': {'right': 'close'}},                                           # grasp
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': TOP_DOWN}},  # lift - straight up
    {'hold': 3.0, 'right': {'xyz': [*BASKET_XY, BASKET_HOVER_Z], 'quat': TOP_DOWN}},                # transport, hover over the basket
    {'hold': 2.0, 'gripper': {'right': 'open'}},                                            # release - drops into the basket
]

SUCCESS_XY_TOLERANCE = 0.05   # meters - same as TIAGo's, basket geometry is identical
SUCCESS_Z_MAX = 0.60


def check_success(object_xyz, pick_xyz) -> tuple[bool, str]:
    """Identical logic to tiago_control_node's pick_place_basket.check_success() - kept as
    a separate copy (not imported) only because it's colocated with this file's PLAN/task
    constants, matching that file's own convention of keeping a task self-contained."""
    target_x, target_y = BASKET_XY
    fx, fy, fz = object_xyz
    dist = ((fx - target_x) ** 2 + (fy - target_y) ** 2) ** 0.5
    success = dist < SUCCESS_XY_TOLERANCE and fz < SUCCESS_Z_MAX
    message = (
        f"Object ended at ({fx:.3f}, {fy:.3f}, {fz:.3f}), basket at ({target_x:.3f}, {target_y:.3f}), "
        f"dist={dist:.3f} -> {'SUCCESS' if success else 'FAILURE'}"
    )
    return success, message
