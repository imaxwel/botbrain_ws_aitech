#!/usr/bin/python3
"""ROS 2 node that computes the right-arm TCP pose in torso_link frame
using dynamic forward kinematics via PyKDL + URDF.

Subscribes to /joint_states, builds a KDL kinematic chain from torso_link
to right_wrist_yaw_link, runs FK with current joint angles, applies a +X
TCP offset (configurable, default 0.175 m), and outputs xyz (m) + rpy (rad)
at a configurable rate.
"""

import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import PyKDL
import yaml

from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF


# ---------------------------------------------------------------------------
# Inlined replacement for kdl_parser_py.urdf.treeFromUrdfModel
#
# Background: ros-humble does not ship kdl_parser_py as an apt package, and
# the upstream source on the `humble` branch still contains the Python 2
# token `kdl.Joint.None` which is a syntax error on Python 3. Rather than
# vendor a patched copy of the whole package, we inline only what we need.
#
# This is a direct port of kdl_parser_py.urdf.treeFromUrdfModel using the
# Py3-compatible `PyKDL.Joint.Fixed` attribute available in python3-pykdl
# (orocos-kdl 1.5.x).
# ---------------------------------------------------------------------------
def _kdl_pose(pose):
    rpy = pose.rpy if pose and pose.rpy and len(pose.rpy) == 3 else [0.0, 0.0, 0.0]
    xyz = pose.xyz if pose and pose.xyz and len(pose.xyz) == 3 else [0.0, 0.0, 0.0]
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


def _kdl_inertia(i):
    origin = _kdl_pose(i.origin)
    inertia = i.inertia
    return origin.M * PyKDL.RigidBodyInertia(
        i.mass, origin.p,
        PyKDL.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz,
                                inertia.ixy, inertia.ixz, inertia.iyz))


def _kdl_joint(jnt):
    F = _kdl_pose(jnt.origin)
    if jnt.type in ('revolute', 'continuous'):
        return PyKDL.Joint(jnt.name, F.p, F.M * PyKDL.Vector(*jnt.axis),
                           PyKDL.Joint.RotAxis)
    if jnt.type == 'prismatic':
        return PyKDL.Joint(jnt.name, F.p, F.M * PyKDL.Vector(*jnt.axis),
                           PyKDL.Joint.TransAxis)
    # 'fixed', 'floating', 'planar', 'unknown' -> treated as fixed
    return PyKDL.Joint(jnt.name, PyKDL.Joint.Fixed)


def _add_children_to_tree(robot_model, root, tree):
    inert = PyKDL.RigidBodyInertia(0)
    if root.inertial:
        inert = _kdl_inertia(root.inertial)

    parent_joint_name, parent_link_name = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]

    sgm = PyKDL.Segment(
        root.name,
        _kdl_joint(parent_joint),
        _kdl_pose(parent_joint.origin),
        inert)

    if not tree.addSegment(sgm, parent_link_name):
        return False

    if root.name not in robot_model.child_map:
        return True

    for _joint_name, child_link_name in robot_model.child_map[root.name]:
        child = robot_model.link_map[child_link_name]
        if not _add_children_to_tree(robot_model, child, tree):
            return False
    return True


def treeFromUrdfModel(robot_model, quiet=True):
    """Construct a PyKDL.Tree from a urdf_parser_py URDF model."""
    root = robot_model.link_map[robot_model.get_root()]
    if root.inertial and not quiet:
        print(f"The root link {root.name} has an inertia specified in the URDF, "
              f"but KDL does not support a root link with an inertia.")

    tree = PyKDL.Tree(root.name)
    ok = True
    for _joint_name, child_link_name in robot_model.child_map[root.name]:
        child = robot_model.link_map[child_link_name]
        if not _add_children_to_tree(robot_model, child, tree):
            ok = False
            break
    return (ok, tree)


class TcpTorsoPoseNode(Node):
    def __init__(self):
        super().__init__('tcp_torso_pose')

        # ---------- parameters ----------
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('tcp_offset_x', self._load_default_tcp_offset_x())
        self.declare_parameter('base_link', 'torso_link')
        self.declare_parameter('tip_link', 'right_wrist_yaw_link')
        self.declare_parameter('publish_rate', 10.0)

        urdf_path = self.get_parameter('urdf_path').value
        self.tcp_offset_x = self.get_parameter('tcp_offset_x').value
        base_link = self.get_parameter('base_link').value
        tip_link = self.get_parameter('tip_link').value
        publish_rate = self.get_parameter('publish_rate').value

        # ---------- load URDF ----------
        if urdf_path:
            self.get_logger().info(f'Loading URDF from file: {urdf_path}')
            with open(urdf_path, 'rb') as f:
                urdf_xml = f.read()
            robot_urdf = URDF.from_xml_string(urdf_xml)
        else:
            # Try /robot_description param first
            urdf_xml = self._try_robot_description()
            if urdf_xml is None:
                # Fall back to default URDF
                from ament_index_python import get_package_share_directory
                share_dir = get_package_share_directory('unitree_g1_dex3_stack')
                default_urdf = os.path.join(
                    share_dir, 'robots', 'g1_description',
                    'g1_29dof_lock_waist_with_hand_rev_1_0.urdf')
                self.get_logger().info(
                    f'/robot_description unavailable, loading default URDF: {default_urdf}')
                with open(default_urdf, 'rb') as f:
                    urdf_xml = f.read()
            # urdf_parser_py uses lxml under the hood, which rejects unicode
            # strings carrying an encoding declaration ("<?xml ... encoding=...?>").
            # Always feed it bytes to be safe.
            if isinstance(urdf_xml, str):
                urdf_xml = urdf_xml.encode('utf-8')
            robot_urdf = URDF.from_xml_string(urdf_xml)

        # ---------- build KDL tree ----------
        ok, kdl_tree = treeFromUrdfModel(robot_urdf)
        if not ok:
            self.get_logger().fatal('Failed to build KDL tree from URDF')
            sys.exit(1)

        # ---------- extract chain ----------
        # PyKDL.Tree.getChain returns a PyKDL.Chain directly (not a tuple).
        try:
            chain = kdl_tree.getChain(base_link, tip_link)
        except Exception as e:
            self.get_logger().fatal(
                f'Failed to get KDL chain from "{base_link}" to "{tip_link}": {e}')
            sys.exit(1)
        if chain.getNrOfSegments() == 0:
            self.get_logger().fatal(
                f'Empty KDL chain from "{base_link}" to "{tip_link}"')
            sys.exit(1)
        # IMPORTANT: keep references to tree and chain on self.
        # PyKDL solvers (ChainFkSolverPos_recursive) only store the C++ pointer
        # to the chain, not a Python ref. If the local Python object goes out
        # of scope and gets garbage-collected, the solver ends up reading freed
        # memory and JntToCart() starts returning -4 (E_SIZE_MISMATCH).
        self.kdl_tree = kdl_tree
        self.kdl_chain = chain

        # ---------- build ordered joint name list ----------
        self.kdl_joint_names = []
        for i in range(chain.getNrOfSegments()):
            seg = chain.getSegment(i)
            joint = seg.getJoint()
            if joint.getTypeName() != 'None':
                self.kdl_joint_names.append(joint.getName())

        self.get_logger().info(
            f'KDL chain: {base_link} -> {tip_link} '
            f'({chain.getNrOfJoints()} joints)')
        self.get_logger().info(
            f'Chain joint names: {self.kdl_joint_names}')

        # ---------- FK solver ----------
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        # ---------- state ----------
        self.current_joints = None
        self.got_first_state = False
        self._waiting_printed = False

        # ---------- ROS I/O ----------
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def _load_default_tcp_offset_x(self):
        try:
            config_path = os.path.join(
                get_package_share_directory('unitree_g1_dex3_stack'),
                'config',
                'apriltag.yaml')
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            value = data.get('tcp_torso_pose', {}).get(
                'ros__parameters', {}).get('tcp_offset_x')
            if value is not None:
                return float(value)
        except Exception as e:
            self.get_logger().warn(
                f'Failed to load tcp_offset_x from config/apriltag.yaml: {e}')
        return 0.175

    # ------------------------------------------------------------------
    def _try_robot_description(self):
        """Try to get URDF XML from the /robot_description parameter.

        Returns the URDF XML string, or None if unavailable.
        """
        try:
            from rclpy.parameter import Parameter
            desc_param = self.get_parameter('robot_description')
            if desc_param.type_ == Parameter.Type.STRING and desc_param.value:
                self.get_logger().info('Loaded URDF from /robot_description parameter')
                return desc_param.value
        except Exception:
            pass
        return None

    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        """Store the latest joint angles ordered to match the KDL chain."""
        joint_array = PyKDL.JntArray(len(self.kdl_joint_names))
        msg_name_to_idx = {name: i for i, name in enumerate(msg.name)}

        for i, jn in enumerate(self.kdl_joint_names):
            idx = msg_name_to_idx.get(jn)
            if idx is not None and idx < len(msg.position):
                joint_array[i] = msg.position[idx]
            else:
                joint_array[i] = 0.0

        self.current_joints = joint_array
        self.got_first_state = True

    # ------------------------------------------------------------------
    def timer_callback(self):
        """Compute FK and print TCP pose in torso_link frame."""
        if not self.got_first_state:
            if not self._waiting_printed:
                self.get_logger().info('Waiting for /joint_states...')
                self._waiting_printed = True
            return
        self._waiting_printed = False

        fk_result = PyKDL.Frame()
        fk_ret = self.fk_solver.JntToCart(self.current_joints, fk_result)
        if fk_ret < 0:
            self.get_logger().error(f'FK solver failed: ret={fk_ret}')
            return

        # Apply TCP offset: +X in wrist_yaw frame
        tcp_offset = PyKDL.Frame(PyKDL.Vector(self.tcp_offset_x, 0.0, 0.0))
        tcp_frame = fk_result * tcp_offset

        x, y, z = tcp_frame.p.x(), tcp_frame.p.y(), tcp_frame.p.z()
        r, p, yw = tcp_frame.M.GetRPY()

        self.get_logger().info(
            f'TCP in torso_link: '
            f'xyz=[{x:.4f}, {y:.4f}, {z:.4f}] m, '
            f'rpy=[{r:.4f}, {p:.4f}, {yw:.4f}] rad')


def main(args=None):
    rclpy.init(args=args)
    node = TcpTorsoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
