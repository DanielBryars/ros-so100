"""Shared pieces for the Linux port of the thesis VR scripts.

Ported from lerobot-thesis/scripts/recording/{vr_openxr_viewer,teleop_sim_vr}.py, which only
ran on Windows + Quest Link. This version runs on Linux + WiVRn/Monado (and keeps the Win32
binding), using the session handling proven in xr_render_linux.py:

  * Xlib/GLX graphics binding fed from the current GLFW context
  * begin_session only on the READY event, end_session on STOPPING
  * render/submit layers only when should_render, otherwise submit an empty frame
  * xr.sync_actions raising SessionNotFocused is not an error

Self-contained in this repo: the scene XML is in scenes/, its meshes are the ones in
ros2_ws/src/so101_description/meshes, and the joint limits are below. lerobot-thesis is not
needed at run time.

On this desktop the monitor is on the iGPU, so run with
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
(see docs/desktop-setup.md for the full env block when launching over SSH).
"""
import ctypes
import os
import sys
import threading
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np
import xr
import xr.exception
from OpenGL import GL

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENES_DIR = REPO_ROOT / "scenes"
DEFAULT_SCENE = "so101_with_wrist_cam.xml"
DEFAULT_PORT = "COM8" if sys.platform == "win32" else "/dev/ttyACM0"


# Joint order and sim joint limits in radians (copied from lerobot-thesis utils/constants.py;
# they match the actuator ctrlrange values in scenes/so101_with_wrist_cam.xml).
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SIM_ACTION_LOW = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.17453])
SIM_ACTION_HIGH = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121, 1.74533])
GRIPPER_IDX = 5


def normalized_to_radians(normalized):
    """Joints [-100, 100] and gripper [0, 100] -> radians (same mapping as thesis utils.conversions)."""
    n = np.asarray(normalized, dtype=np.float64)
    t = (n + 100.0) / 200.0
    t[GRIPPER_IDX] = n[GRIPPER_IDX] / 100.0
    return SIM_ACTION_LOW + t * (SIM_ACTION_HIGH - SIM_ACTION_LOW)


def matte_black_floor(model, geom_name="floor"):
    """Make the ground plane plain black: no checker texture, no reflection, no highlights.
    Same look as the thesis's *_matte_black_floor scene, applied to whichever scene is loaded."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if gid < 0:
        print(f"No geom named '{geom_name}'; floor left as it is")
        return
    mid = model.geom_matid[gid]
    if mid < 0:
        model.geom_rgba[gid] = [0.02, 0.02, 0.02, 1.0]
        return
    model.mat_texid[mid, :] = -1
    model.mat_rgba[mid] = [0.02, 0.02, 0.02, 1.0]
    model.mat_reflectance[mid] = 0.0
    model.mat_specular[mid] = 0.0
    model.mat_shininess[mid] = 0.0


def load_scene(name=DEFAULT_SCENE, settle_steps=100, floor="black"):
    """Load a MuJoCo scene (by file name in scenes/, or a path) and let it settle. Returns (model, data).

    floor="black" replaces the reflective checker ground with matte black; "scene" keeps
    whatever the XML defines.
    """
    path = Path(name)
    if not path.exists():
        path = SCENES_DIR / name
    print(f"Loading MuJoCo scene: {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    if floor == "black":
        matte_black_floor(model)
    data = mujoco.MjData(model)
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    return model, data


def arm_qpos_indices(model):
    """qpos addresses of the six arm joints, by name (falls back to the first six)."""
    idx = []
    for name in MOTOR_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            return list(range(6))
        idx.append(int(model.jnt_qposadr[jid]))
    return idx


# --------------------------------------------------------------------------------------
# Leader / follower arm reader
# --------------------------------------------------------------------------------------

class ArmReader:
    """Reads a physical SO100 arm (STS3250) in a background thread so serial I/O never
    blocks the 90 Hz VR loop. `latest()` returns (joint_radians or None, age_seconds)."""

    def __init__(self, port, is_follower=False, hz=200):
        self.port = port
        self.is_follower = is_follower
        self.hz = hz
        self.bus = None
        self.failed = False
        self._lock = threading.Lock()
        self._radians = None
        self._stamp = 0.0
        self._run = False
        self._thread = None

    def connect(self):
        import draccus
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

        kind = "follower" if self.is_follower else "leader"
        print(f"Connecting to {kind} arm on {self.port}...")
        motors = {name: Motor(i + 1, "sts3250",
                              MotorNormMode.RANGE_0_100 if i == GRIPPER_IDX else MotorNormMode.RANGE_M100_100)
                  for i, name in enumerate(MOTOR_NAMES)}
        self.bus = FeetechMotorsBus(port=self.port, motors=motors)
        self.bus.connect()

        sub = ("robots/so100_follower_sts3250/follower_so100.json" if self.is_follower
               else "teleoperators/so100_leader_sts3250/leader_so100.json")
        calib_path = HF_LEROBOT_CALIBRATION / sub
        if not calib_path.exists():
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        with open(calib_path) as f, draccus.config_type("json"):
            self.bus.calibration = draccus.load(dict[str, MotorCalibration], f)
        self.bus.disable_torque()

        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"{kind.capitalize()} arm connected; move it to control the simulation.")

    def _loop(self):
        errors = 0
        interval = 1.0 / self.hz
        while self._run:
            t0 = time.perf_counter()
            try:
                pos = self.bus.sync_read("Present_Position")
                rad = normalized_to_radians([pos[n] for n in MOTOR_NAMES])
                rad = np.clip(rad, SIM_ACTION_LOW, SIM_ACTION_HIGH)
                with self._lock:
                    self._radians, self._stamp = rad, time.perf_counter()
                if errors:
                    print(f"Arm reconnected after {errors} failed reads")
                errors = 0
            except Exception as e:  # serial hiccups are routine; keep the last pose
                errors += 1
                if errors == 1:
                    print(f"Arm read failed ({e}); holding last position...")
                if errors >= 100:
                    print("Too many consecutive arm read errors; giving up on the arm.")
                    self.failed = True
                    return
                time.sleep(0.02)
                continue
            dt = time.perf_counter() - t0
            if dt < interval:
                time.sleep(interval - dt)

    def latest(self):
        with self._lock:
            if self._radians is None:
                return None, 0.0
            return self._radians.copy(), time.perf_counter() - self._stamp

    def close(self):
        self._run = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.bus is not None:
            self.bus.disconnect()
            self.bus = None


# --------------------------------------------------------------------------------------
# OpenXR renderer
# --------------------------------------------------------------------------------------

def _s(x):
    return x.decode() if isinstance(x, bytes) else x


def _graphics_binding():
    """OpenXR graphics binding for the GL context that is current on this thread."""
    if sys.platform == "win32":
        from OpenGL import WGL
        return xr.GraphicsBindingOpenGLWin32KHR(h_dc=WGL.wglGetCurrentDC(), h_glrc=WGL.wglGetCurrentContext())

    from OpenGL import GLX
    fields = dict(xr.GraphicsBindingOpenGLXlibKHR._fields_)

    def coerce(ptr, name):
        # pyopenxr declares typed pointers (LP_struct__XDisplay, ...); PyOpenGL hands back its own.
        t = fields[name]
        if isinstance(t, type) and issubclass(t, ctypes._Pointer):
            return ctypes.cast(ptr, t) if ptr else t()
        return ctypes.cast(ptr, ctypes.c_void_p).value or 0

    display = GLX.glXGetCurrentDisplay()
    if not display:
        raise RuntimeError("No current GLX context. GLFW must be on X11/XWayland (DISPLAY set), not native Wayland.")
    return xr.GraphicsBindingOpenGLXlibKHR(
        x_display=coerce(display, "x_display"),
        visualid=0,
        glx_fbconfig=coerce(None, "glx_fbconfig"),
        glx_drawable=GLX.glXGetCurrentDrawable(),
        glx_context=coerce(GLX.glXGetCurrentContext(), "glx_context"),
    )


def _quat_rotate(q, v):
    """Rotate v by an OpenXR quaternion (x, y, z, w)."""
    qv = np.array([q.x, q.y, q.z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    t = 2.0 * np.cross(qv, v)
    return v + q.w * t + np.cross(qv, t)


def _xr_to_mj(v):
    """OpenXR (+X right, +Y up, -Z forward) -> MuJoCo (+X forward, +Y left, +Z up)."""
    return np.array([-v[2], -v[0], v[1]], dtype=np.float64)


def _rotate_z(v, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]], dtype=np.float64)


class VRRenderer:
    """Renders a MuJoCo model into an OpenXR headset.

    Same public surface as the thesis VRRenderer (init_all, render_frame, check_keyboard,
    cleanup, base_pos, view_yaw, show_*_cam_fov), so callers written against it keep working.

    Controls
      left stick / right stick   move / yaw and up-down
      X or A                     recentre now: robot in front of where you are looking
      Y or B                     recentre in RECENTER_DELAY seconds, so you can put the
                                 controllers down and get into position at the real arm first
      thumbstick click           put the sim robot's base where that controller is (rest the
                                 controller on the real arm's base, pointing the way the arm
                                 faces), so the virtual arm sits on top of the real one
    Desktop window (needs keyboard focus): WASD/QE = move, R or Space = recentre,
    T = delayed recentre, P = print pose.
    """

    MOVE_SPEED = 0.6   # m/s at full stick deflection
    YAW_SPEED = 0.9    # rad/s
    # Eye position relative to the robot base after a recentre (MuJoCo frame, yaw-aligned).
    RECENTER_EYE_POS = np.array([-0.2, 0.0, 0.55])
    RECENTER_DELAY = 5.0  # seconds, for the Y/B/T delayed recentre

    def __init__(self, mj_model, mj_data, mirror=True, auto_recenter=True):
        self.model = mj_model
        self.data = mj_data
        self.mirror = mirror
        self.auto_recenter = auto_recenter

        self.mj_scene = None
        self.mj_context = None
        self.mj_option = mujoco.MjvOption()

        self.window = None
        self.instance = None
        self.session = None
        self.swapchains = []
        self.frame_buffers = []   # per eye: list of FBO ids, one per swapchain image
        self.sizes = []
        self.reference_space = None

        self.session_state = xr.SessionState.UNKNOWN
        self.session_running = False
        self.quit_requested = False
        self.is_focused = False
        self.tracking_valid = False
        self.frames_rendered = 0
        self.frame_dt = 0.0

        self.base_pos = np.array([0.4, 0.3, 0.5], dtype=np.float64)
        self.view_yaw = 0.0
        self.last_head_pos = None
        self.last_head_fwd = None

        self.show_wrist_cam_fov = False
        self.show_overhead_cam_fov = False

        self.snapshot_path = None   # set to a file path to dump the next left-eye image
        self._last_frame_t = None
        self._keys_down = set()
        self._recentered_once = False
        self._recenter_at = None      # perf_counter deadline of a pending delayed recentre
        self._countdown_shown = None

    # ---- init ----

    def init_all(self):
        self._init_glfw()
        self._init_openxr()
        self._init_session()
        self._init_actions()
        self._init_swapchains()
        self._init_mujoco_rendering()
        self._init_reference_space()

    def _init_glfw(self):
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_COMPAT_PROFILE)  # MuJoCo needs compat
        glfw.window_hint(glfw.DOUBLEBUFFER, True)
        glfw.window_hint(glfw.VISIBLE, True)
        self.window = glfw.create_window(800, 838, "MuJoCo VR (left eye mirror)", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)  # the headset paces us via xrWaitFrame, never the monitor
        renderer = _s(GL.glGetString(GL.GL_RENDERER))
        print(f"GL: {_s(GL.glGetString(GL.GL_VERSION))} | {renderer}")
        if sys.platform != "win32" and "nvidia" not in renderer.lower() and "rtx" not in renderer.lower():
            print("WARNING: GL context is not on the NVIDIA GPU. Set __NV_PRIME_RENDER_OFFLOAD=1 "
                  "__GLX_VENDOR_LIBRARY_NAME=nvidia (see docs/desktop-setup.md).")

    def _init_openxr(self):
        self.instance = xr.create_instance(xr.InstanceCreateInfo(
            enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]))
        print(f"OpenXR runtime: {_s(xr.get_instance_properties(self.instance).runtime_name)}")
        self.system_id = xr.get_system(self.instance, xr.SystemGetInfo(
            form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
        self.view_config_type = xr.ViewConfigurationType.PRIMARY_STEREO
        self.view_config_views = xr.enumerate_view_configuration_views(
            self.instance, self.system_id, self.view_config_type)

    def _init_session(self):
        # xrGetOpenGLGraphicsRequirementsKHR must be called before xrCreateSession
        pfn = ctypes.cast(
            xr.get_instance_proc_addr(self.instance, "xrGetOpenGLGraphicsRequirementsKHR"),
            xr.PFN_xrGetOpenGLGraphicsRequirementsKHR)
        req = xr.GraphicsRequirementsOpenGLKHR()
        pfn(self.instance, self.system_id, ctypes.byref(req))

        self._binding = _graphics_binding()  # keep alive for the session's lifetime
        self.session = xr.create_session(self.instance, xr.SessionCreateInfo(
            system_id=self.system_id,
            next=ctypes.cast(ctypes.pointer(self._binding), ctypes.c_void_p)))

    def _init_actions(self):
        inst = self.instance
        path = lambda s: xr.string_to_path(inst, s)
        self.action_set = xr.create_action_set(inst, xr.ActionSetCreateInfo(
            action_set_name="scene_control", localized_action_set_name="Scene Control", priority=0))
        self.hand_paths = (xr.Path * 2)(path("/user/hand/left"), path("/user/hand/right"))

        def action(name, label, kind):
            return xr.create_action(self.action_set, xr.ActionCreateInfo(
                action_name=name, action_type=kind, localized_action_name=label,
                count_subaction_paths=2, subaction_paths=self.hand_paths))

        self.stick_x_action = action("thumbstick_x", "Thumbstick X", xr.ActionType.FLOAT_INPUT)
        self.stick_y_action = action("thumbstick_y", "Thumbstick Y", xr.ActionType.FLOAT_INPUT)
        self.recenter_action = action("recenter", "Recenter Scene", xr.ActionType.BOOLEAN_INPUT)
        self.delayed_action = action("recenter_delayed", "Recenter After Delay", xr.ActionType.BOOLEAN_INPUT)
        self.place_action = action("place_robot", "Place Robot At Controller", xr.ActionType.BOOLEAN_INPUT)
        self.grip_action = action("grip_pose", "Grip Pose", xr.ActionType.POSE_INPUT)
        self.haptic_action = action("haptic", "Haptic", xr.ActionType.VIBRATION_OUTPUT)

        bindings = [
            xr.ActionSuggestedBinding(self.stick_x_action, path("/user/hand/left/input/thumbstick/x")),
            xr.ActionSuggestedBinding(self.stick_y_action, path("/user/hand/left/input/thumbstick/y")),
            xr.ActionSuggestedBinding(self.stick_x_action, path("/user/hand/right/input/thumbstick/x")),
            xr.ActionSuggestedBinding(self.stick_y_action, path("/user/hand/right/input/thumbstick/y")),
            xr.ActionSuggestedBinding(self.recenter_action, path("/user/hand/left/input/x/click")),
            xr.ActionSuggestedBinding(self.recenter_action, path("/user/hand/right/input/a/click")),
            xr.ActionSuggestedBinding(self.delayed_action, path("/user/hand/left/input/y/click")),
            xr.ActionSuggestedBinding(self.delayed_action, path("/user/hand/right/input/b/click")),
        ]
        for hand in ("left", "right"):
            bindings += [
                xr.ActionSuggestedBinding(self.place_action, path(f"/user/hand/{hand}/input/thumbstick/click")),
                xr.ActionSuggestedBinding(self.grip_action, path(f"/user/hand/{hand}/input/grip/pose")),
                xr.ActionSuggestedBinding(self.haptic_action, path(f"/user/hand/{hand}/output/haptic")),
            ]
        arr = (xr.ActionSuggestedBinding * len(bindings))(*bindings)
        try:
            xr.suggest_interaction_profile_bindings(inst, xr.InteractionProfileSuggestedBinding(
                interaction_profile=path("/interaction_profiles/oculus/touch_controller"),
                count_suggested_bindings=len(bindings), suggested_bindings=arr))
        except Exception as e:
            print(f"Warning: could not bind Oculus Touch profile: {e}")
        xr.attach_session_action_sets(self.session, xr.SessionActionSetsAttachInfo(
            count_action_sets=1, action_sets=ctypes.pointer(self.action_set)))
        self.hand_spaces = [xr.create_action_space(self.session, xr.ActionSpaceCreateInfo(
            action=self.grip_action, subaction_path=p)) for p in self.hand_paths]

    def _init_swapchains(self):
        glfw.make_context_current(self.window)
        formats = xr.enumerate_swapchain_formats(self.session)
        fmt = GL.GL_SRGB8_ALPHA8 if GL.GL_SRGB8_ALPHA8 in formats else formats[0]
        for i, vc in enumerate(self.view_config_views):
            w, h = vc.recommended_image_rect_width, vc.recommended_image_rect_height
            sc = xr.create_swapchain(self.session, xr.SwapchainCreateInfo(
                usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT | xr.SwapchainUsageFlags.SAMPLED_BIT,
                format=fmt, sample_count=1, width=w, height=h, face_count=1, array_size=1, mip_count=1))
            images = xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)

            depth = int(GL.glGenRenderbuffers(1))  # one depth buffer per eye, shared by its images
            GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, depth)
            GL.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL.GL_DEPTH24_STENCIL8, w, h)
            fbos = []
            for img in images:
                fbo = int(GL.glGenFramebuffers(1))
                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
                GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                                          GL.GL_TEXTURE_2D, img.image, 0)
                GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_STENCIL_ATTACHMENT,
                                             GL.GL_RENDERBUFFER, depth)
                status = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
                if status != GL.GL_FRAMEBUFFER_COMPLETE:
                    print(f"Warning: framebuffer incomplete: {status:#x}")
                fbos.append(fbo)
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
            self.swapchains.append(sc)
            self.frame_buffers.append(fbos)
            self.sizes.append((w, h))
            print(f"  Eye {i}: {w}x{h}, {len(images)} images, format {fmt:#x}")

    def _init_mujoco_rendering(self):
        self.mj_scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.mj_context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

    def _init_reference_space(self):
        # The session is NOT begun here: Monado/WiVRn require waiting for READY, which
        # _poll_session_events handles from render_frame.
        try:
            supported = list(xr.enumerate_reference_spaces(self.session))
        except Exception:
            supported = []
        for space_type in (xr.ReferenceSpaceType.STAGE, xr.ReferenceSpaceType.LOCAL):
            if supported and space_type not in supported:
                continue
            try:
                self.reference_space = xr.create_reference_space(self.session, xr.ReferenceSpaceCreateInfo(
                    reference_space_type=space_type,
                    pose_in_reference_space=xr.Posef(orientation=xr.Quaternionf(0, 0, 0, 1),
                                                     position=xr.Vector3f(0, 0, 0))))
                print(f"Using reference space: {space_type.name}")
                break
            except Exception as e:
                print(f"Reference space {space_type.name} failed: {e}")
        if self.reference_space is None:
            raise RuntimeError("Could not create an OpenXR reference space (STAGE/LOCAL)")
        print("Waiting for the headset (session begins when the runtime reports READY)...")

    # ---- session lifecycle ----

    def _poll_session_events(self):
        while True:
            try:
                ev = xr.poll_event(self.instance)
            except xr.exception.EventUnavailable:
                return
            if ev.type != xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                continue
            changed = ctypes.cast(ctypes.byref(ev), ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
            self.session_state = xr.SessionState(changed.state)
            self.is_focused = self.session_state == xr.SessionState.FOCUSED
            print(f"Session state: {self.session_state.name}")
            if self.session_state == xr.SessionState.READY:
                xr.begin_session(self.session, xr.SessionBeginInfo(
                    primary_view_configuration_type=self.view_config_type))
                self.session_running = True
            elif self.session_state == xr.SessionState.STOPPING:
                xr.end_session(self.session)
                self.session_running = False
            elif self.session_state in (xr.SessionState.EXITING, xr.SessionState.LOSS_PENDING):
                self.session_running = False
                self.quit_requested = True

    # ---- input ----

    def _stick(self, action, hand):
        st = xr.get_action_state_float(self.session, xr.ActionStateGetInfo(
            action=action, subaction_path=self.hand_paths[hand]))
        return st.current_state if st.is_active and abs(st.current_state) > 0.1 else 0.0

    def _pressed(self, action, hand=None):
        st = xr.get_action_state_boolean(self.session, xr.ActionStateGetInfo(
            action=action, subaction_path=xr.NULL_PATH if hand is None else self.hand_paths[hand]))
        return bool(st.is_active and st.current_state and st.changed_since_last_sync)

    def _buzz(self, seconds=0.1, amplitude=0.6):
        """Short vibration on both controllers (feedback you can feel with the headset on)."""
        vib = xr.HapticVibration(duration=int(seconds * 1e9), frequency=xr.FREQUENCY_UNSPECIFIED,
                                 amplitude=amplitude)
        for p in self.hand_paths:
            try:
                xr.apply_haptic_feedback(self.session, xr.HapticActionInfo(
                    action=self.haptic_action, subaction_path=p),
                    ctypes.cast(ctypes.pointer(vib), ctypes.POINTER(xr.HapticBaseHeader)).contents)
            except Exception:
                pass

    def request_delayed_recenter(self):
        self._recenter_at = time.perf_counter() + self.RECENTER_DELAY
        self._countdown_shown = None
        print(f"Recentring in {self.RECENTER_DELAY:.0f} s: get into position and look at where the robot should be")

    def _tick_delayed_recenter(self):
        if self._recenter_at is None:
            return
        left = self._recenter_at - time.perf_counter()
        if left <= 0:
            self._recenter_at = None
            self._recenter_scene()
            if self.is_focused:
                self._buzz(0.4)
        elif int(left) != self._countdown_shown:
            self._countdown_shown = int(left)
            print(f"  ...{int(left) + 1}")
            if self.is_focused:
                self._buzz(0.05, 0.3)

    def _place_robot_at_controller(self, hand, display_time):
        """Move the scene so the robot base (MuJoCo origin) sits at this controller, facing
        the way the controller points."""
        loc = xr.locate_space(self.hand_spaces[hand], self.reference_space, display_time)
        need = xr.SPACE_LOCATION_POSITION_VALID_BIT | xr.SPACE_LOCATION_ORIENTATION_VALID_BIT
        if (int(loc.location_flags) & int(need)) != int(need):
            print("Cannot place robot: controller is not tracked")
            return
        p = loc.pose.position
        fwd_mj = _xr_to_mj(_quat_rotate(loc.pose.orientation, [0.0, 0.0, -1.0]))
        self.view_yaw = -np.arctan2(fwd_mj[1], fwd_mj[0])
        # world = rotate_z(base_pos + xr_to_mj(p), yaw) must be the origin  =>  base_pos = -xr_to_mj(p)
        self.base_pos = -_xr_to_mj([p.x, p.y, p.z])
        self._recenter_at = None
        self._buzz(0.2)
        print(f"Robot base placed at the {'left' if hand == 0 else 'right'} controller: "
              f"base_pos {np.round(self.base_pos, 2).tolist()}, yaw {np.degrees(self.view_yaw):.1f} deg")

    def _handle_controller_input(self, dt, display_time):
        try:
            xr.sync_actions(self.session, xr.ActionsSyncInfo(
                count_active_action_sets=1,
                active_action_sets=ctypes.pointer(xr.ActiveActionSet(self.action_set, xr.NULL_PATH))))
        except xr.exception.SessionNotFocused:
            return  # a success code in the spec; pyopenxr raises it. No input while unfocused.
        if not self.is_focused:
            return
        self.base_pos[1] -= self._stick(self.stick_x_action, 0) * self.MOVE_SPEED * dt
        self.base_pos[0] += self._stick(self.stick_y_action, 0) * self.MOVE_SPEED * dt
        self.view_yaw += self._stick(self.stick_x_action, 1) * self.YAW_SPEED * dt
        self.base_pos[2] += self._stick(self.stick_y_action, 1) * self.MOVE_SPEED * dt
        if self._pressed(self.recenter_action):
            self._recenter_at = None
            self._recenter_scene()
            self._buzz(0.2)
        if self._pressed(self.delayed_action):
            self.request_delayed_recenter()
        for hand in (0, 1):
            if self._pressed(self.place_action, hand):
                self._place_robot_at_controller(hand, display_time)

    def _key_pressed_once(self, key):
        down = glfw.get_key(self.window, key) == glfw.PRESS
        was = key in self._keys_down
        (self._keys_down.add if down else self._keys_down.discard)(key)
        return down and not was

    def check_keyboard(self, dt=None):
        """Keyboard controls on the desktop mirror window (replaces the msvcrt console keys)."""
        if self.window is None:
            return
        dt = self.frame_dt if dt is None else dt
        step = self.MOVE_SPEED * dt
        held = lambda k: glfw.get_key(self.window, k) == glfw.PRESS
        self.base_pos[0] += step * (held(glfw.KEY_W) - held(glfw.KEY_S))
        self.base_pos[1] += step * (held(glfw.KEY_A) - held(glfw.KEY_D))
        self.base_pos[2] += step * (held(glfw.KEY_Q) - held(glfw.KEY_E))
        if self._key_pressed_once(glfw.KEY_R) or self._key_pressed_once(glfw.KEY_SPACE):
            self._recenter_at = None
            self._recenter_scene()
        if self._key_pressed_once(glfw.KEY_T):
            self.request_delayed_recenter()
        if self._key_pressed_once(glfw.KEY_P):
            print(f"base_pos: {np.round(self.base_pos, 3).tolist()}, yaw: {np.degrees(self.view_yaw):.1f} deg")

    def _recenter_scene(self):
        """Put the robot ~20 cm in front of the user at bench height, facing them."""
        if self.last_head_pos is None:
            print("Cannot recentre: no head pose yet")
            return
        head_mj = _xr_to_mj(self.last_head_pos)
        fwd_mj = _xr_to_mj(self.last_head_fwd)
        self.view_yaw = -np.arctan2(fwd_mj[1], fwd_mj[0])
        # eye = rotate_z(base_pos + head, yaw)  =>  base_pos = rotate_z(desired_eye, -yaw) - head
        self.base_pos = _rotate_z(self.RECENTER_EYE_POS, -self.view_yaw) - head_mj
        print(f"Scene recentred: base_pos {np.round(self.base_pos, 2).tolist()}, "
              f"yaw {np.degrees(self.view_yaw):.1f} deg")

    # ---- rendering ----

    def _add_cam_fov_overlay(self, cam_name, outline_rgba, fill_rgba, n_fill):
        """Draw a camera's footprint on the table (VR only; never in sim observations)."""
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
        if cam_id < 0:
            return
        cam_pos = self.data.cam_xpos[cam_id].copy()
        cam_mat = self.data.cam_xmat[cam_id].reshape(3, 3)
        ty = np.tan(np.radians(self.model.cam_fovy[cam_id]) / 2)
        tx = ty * 640.0 / 480.0
        table_z = 0.015
        corners = []
        for ray in ([-tx, -ty, -1.0], [tx, -ty, -1.0], [tx, ty, -1.0], [-tx, ty, -1.0]):
            ray_world = cam_mat @ np.array(ray)
            if abs(ray_world[2]) < 1e-6:
                continue
            t = (table_z - cam_pos[2]) / ray_world[2]
            if t > 0:
                corners.append(cam_pos + t * ray_world)
        n = len(corners)
        if n < 3:
            return

        def line(a, b, width, rgba):
            if self.mj_scene.ngeom >= self.mj_scene.maxgeom:
                return
            geom = self.mj_scene.geoms[self.mj_scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.asarray(rgba, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
            self.mj_scene.ngeom += 1

        for i in range(n):
            line(corners[i], corners[(i + 1) % n], 0.005, outline_rgba)
        for j in range(1, n_fill):
            t = j / n_fill
            line(corners[0] * (1 - t) + corners[min(3, n - 1)] * t,
                 corners[1] * (1 - t) + corners[min(2, n - 1)] * t, 0.0025, fill_rgba)

    def render_eye(self, eye_idx, view, fbo, width, height):
        """Render the scene for one eye using its OpenXR pose and asymmetric FOV."""
        q = view.pose.orientation
        p = view.pose.position
        eye_pos = _rotate_z(self.base_pos + _xr_to_mj([p.x, p.y, p.z]), self.view_yaw)
        fwd = _rotate_z(_xr_to_mj(_quat_rotate(q, [0.0, 0.0, -1.0])), self.view_yaw)
        right = _rotate_z(_xr_to_mj(_quat_rotate(q, [1.0, 0.0, 0.0])), self.view_yaw)
        fwd /= np.linalg.norm(fwd) + 1e-12
        right /= np.linalg.norm(right) + 1e-12
        up = np.cross(right, fwd)
        up /= np.linalg.norm(up) + 1e-12

        # A free camera only to populate the scene geoms; its pose is overridden below.
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = eye_pos + fwd
        cam.distance = 1.0
        cam.azimuth = np.degrees(np.arctan2(fwd[1], fwd[0]))
        cam.elevation = np.degrees(np.arctan2(fwd[2], np.hypot(fwd[0], fwd[1])))
        mujoco.mjv_updateScene(self.model, self.data, self.mj_option, None, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, self.mj_scene)
        if self.show_wrist_cam_fov:
            self._add_cam_fov_overlay("wrist_cam", [1.0, 0.15, 0.15, 1.0], [1.0, 0.1, 0.1, 0.35], 12)
        if self.show_overhead_cam_fov:
            self._add_cam_fov_overlay("overhead_cam", [0.2, 0.4, 1.0, 1.0], [0.15, 0.3, 1.0, 0.15], 16)

        near, far = 0.05, 50.0
        fov = view.fov
        left, right_ = near * np.tan(fov.angle_left), near * np.tan(fov.angle_right)
        c = self.mj_scene.camera[0]
        c.pos[:] = eye_pos
        c.forward[:] = fwd
        c.up[:] = up
        c.frustum_near, c.frustum_far = near, far
        c.frustum_center = (left + right_) / 2.0
        c.frustum_width = (right_ - left) / 2.0
        c.frustum_bottom = near * np.tan(fov.angle_down)
        c.frustum_top = near * np.tan(fov.angle_up)
        self.mj_scene.stereo = mujoco.mjtStereo.mjSTEREO_NONE

        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
        GL.glViewport(0, 0, width, height)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.2, 0.3, 0.4, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        mujoco.mjr_render(mujoco.MjrRect(0, 0, width, height), self.mj_scene, self.mj_context)

    def _save_snapshot(self, fbo, width, height):
        import cv2
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, fbo)
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        buf = GL.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        img = np.frombuffer(buf, dtype=np.uint8).reshape(height, width, 3)[::-1]
        cv2.imwrite(str(self.snapshot_path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"Snapshot saved: {self.snapshot_path} (mean pixel {img.mean():.1f}, std {img.std():.1f})")
        self.snapshot_path = None

    def _mirror(self, fbo, width, height):
        ww, wh = glfw.get_framebuffer_size(self.window)
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, fbo)
        GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, 0)
        GL.glBlitFramebuffer(0, 0, width, height, 0, 0, ww, wh, GL.GL_COLOR_BUFFER_BIT, GL.GL_LINEAR)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        glfw.swap_buffers(self.window)

    def render_frame(self):
        """Run one VR frame (blocks in xrWaitFrame at the headset rate). Returns False to exit.

        While the session is not running (headset asleep, before READY) this returns True
        after a short sleep without rendering, so callers can keep looping.
        """
        glfw.make_context_current(self.window)
        glfw.poll_events()
        if glfw.window_should_close(self.window):
            return False
        self._poll_session_events()
        if self.quit_requested:
            return False

        now = time.perf_counter()
        self.frame_dt = min(now - self._last_frame_t, 0.1) if self._last_frame_t else 0.0
        self._last_frame_t = now

        if not self.session_running:
            time.sleep(0.01)
            return True

        frame_state = xr.wait_frame(self.session)
        xr.begin_frame(self.session)
        self._handle_controller_input(self.frame_dt, frame_state.predicted_display_time)

        projection_views = []
        if frame_state.should_render:
            view_state, views = xr.locate_views(self.session, xr.ViewLocateInfo(
                view_configuration_type=self.view_config_type,
                display_time=frame_state.predicted_display_time,
                space=self.reference_space))
            flags = int(view_state.view_state_flags)
            self.tracking_valid = bool(flags & xr.VIEW_STATE_ORIENTATION_VALID_BIT
                                       and flags & xr.VIEW_STATE_POSITION_VALID_BIT)
            q = views[0].pose.orientation
            # Headset in standby reports invalid poses; only render with a usable quaternion.
            if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) < 1e-3:
                p = views[0].pose.position
                self.last_head_pos = np.array([p.x, p.y, p.z])
                self.last_head_fwd = _quat_rotate(q, [0.0, 0.0, -1.0])
                if self.tracking_valid and self.auto_recenter and not self._recentered_once:
                    self._recentered_once = True
                    self._recenter_scene()
                self._tick_delayed_recenter()

                for eye, (view, sc, fbos, (w, h)) in enumerate(
                        zip(views, self.swapchains, self.frame_buffers, self.sizes)):
                    idx = xr.acquire_swapchain_image(sc)
                    xr.wait_swapchain_image(sc, xr.SwapchainImageWaitInfo(timeout=xr.INFINITE_DURATION))
                    self.render_eye(eye, view, fbos[idx], w, h)
                    if eye == 0:
                        if self.snapshot_path:
                            self._save_snapshot(fbos[idx], w, h)
                        if self.mirror:
                            self._mirror(fbos[idx], w, h)
                    GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
                    xr.release_swapchain_image(sc)
                    projection_views.append(xr.CompositionLayerProjectionView(
                        pose=view.pose, fov=view.fov,
                        sub_image=xr.SwapchainSubImage(
                            swapchain=sc, image_array_index=0,
                            image_rect=xr.Rect2Di(offset=xr.Offset2Di(0, 0), extent=xr.Extent2Di(w, h)))))
                self.frames_rendered += 1

        layers = []
        if projection_views:
            pv = (xr.CompositionLayerProjectionView * len(projection_views))(*projection_views)
            layer = xr.CompositionLayerProjection(space=self.reference_space,
                                                  view_count=len(projection_views), views=pv)
            layers = [ctypes.byref(layer)]
        xr.end_frame(self.session, xr.FrameEndInfo(
            display_time=frame_state.predicted_display_time,
            environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE, layers=layers))
        return True

    def cleanup(self):
        if self.session is not None:
            try:
                if self.session_running:
                    xr.request_exit_session(self.session)
                    for _ in range(200):
                        self._poll_session_events()
                        if not self.session_running:
                            break
                        time.sleep(0.01)
                xr.destroy_session(self.session)
            except Exception as e:
                print(f"Session shutdown: {e}")
            self.session = None
        if self.instance is not None:
            try:
                xr.destroy_instance(self.instance)
            except Exception:
                pass
            self.instance = None
        if self.window is not None:
            glfw.destroy_window(self.window)
            glfw.terminate()
            self.window = None


class FpsMeter:
    """Prints a status line every `period` seconds."""

    def __init__(self, period=2.0):
        self.period = period
        self.t = time.perf_counter()
        self.n = 0

    def tick(self, vr, extra=""):
        self.n += 1
        now = time.perf_counter()
        if now - self.t < self.period:
            return
        fps = self.n / (now - self.t)
        self.t, self.n = now, 0
        print(f"VR: {fps:5.1f} loop/s | rendered {vr.frames_rendered} | {vr.session_state.name}"
              f" | tracking {'ok' if vr.tracking_valid else '--'}{extra}", flush=True)
