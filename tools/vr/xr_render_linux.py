"""Linux (X11/GLX) OpenXR render smoke test for WiVRn, mirroring vr_openxr_viewer.py's
structure so the graphics-binding block can be dropped into the thesis viewer.

Runs ~10 s: clears each eye to a colour that follows head yaw, prints head + controller poses.
"""
import ctypes
import math
import sys
import time

import glfw
import xr
import xr.exception
from OpenGL import GL, GLX

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0


def s(x):
    return x.decode() if isinstance(x, bytes) else x


# ---- GLFW / GL context (X11 via XWayland) ----
if not glfw.init():
    raise RuntimeError("glfw.init failed")
glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_COMPAT_PROFILE)
glfw.window_hint(glfw.DOUBLEBUFFER, True)
glfw.window_hint(glfw.VISIBLE, True)
window = glfw.create_window(640, 480, "xr_render_linux", None, None)
if not window:
    raise RuntimeError("glfw.create_window failed")
glfw.make_context_current(window)
glfw.swap_interval(0)
print("GL:", s(GL.glGetString(GL.GL_VERSION)), "|", s(GL.glGetString(GL.GL_RENDERER)))

# ---- OpenXR instance / system ----
instance = xr.create_instance(xr.InstanceCreateInfo(
    enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]))
print("runtime:", s(xr.get_instance_properties(instance).runtime_name))
system_id = xr.get_system(instance, xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
view_cfg = xr.ViewConfigurationType.PRIMARY_STEREO
cfg_views = xr.enumerate_view_configuration_views(instance, system_id, view_cfg)
print("eyes:", [(v.recommended_image_rect_width, v.recommended_image_rect_height) for v in cfg_views])

# xrGetOpenGLGraphicsRequirementsKHR must be called before xrCreateSession
pfn = ctypes.cast(xr.get_instance_proc_addr(instance, "xrGetOpenGLGraphicsRequirementsKHR"),
                  xr.PFN_xrGetOpenGLGraphicsRequirementsKHR)
req = xr.GraphicsRequirementsOpenGLKHR()
pfn(instance, system_id, ctypes.byref(req))
print("GL requirement: min", xr.Version(req.min_api_version_supported), "max", xr.Version(req.max_api_version_supported))

# ---- the Linux replacement for the Win32 binding block ----
F = dict(xr.GraphicsBindingOpenGLXlibKHR._fields_)


def P(ptr, name):
    """Coerce a PyOpenGL/GLX handle into the ctypes type pyopenxr declares for this field."""
    t = F[name]
    if isinstance(t, type) and issubclass(t, ctypes._Pointer):
        return ctypes.cast(ptr, t) if ptr else t()
    return ctypes.cast(ptr, ctypes.c_void_p).value or 0


graphics_binding = xr.GraphicsBindingOpenGLXlibKHR(
    x_display=P(GLX.glXGetCurrentDisplay(), "x_display"),
    visualid=0,
    glx_fbconfig=P(None, "glx_fbconfig"),
    glx_drawable=GLX.glXGetCurrentDrawable(),
    glx_context=P(GLX.glXGetCurrentContext(), "glx_context"),
)
session = xr.create_session(instance, xr.SessionCreateInfo(
    system_id=system_id,
    next=ctypes.cast(ctypes.pointer(graphics_binding), ctypes.c_void_p)))
print("session created")

# ---- swapchains + FBOs ----
formats = xr.enumerate_swapchain_formats(session)
fmt = GL.GL_SRGB8_ALPHA8 if GL.GL_SRGB8_ALPHA8 in formats else formats[0]
swapchains, fbos, sizes = [], [], []
for v in cfg_views:
    w, h = v.recommended_image_rect_width, v.recommended_image_rect_height
    sc = xr.create_swapchain(session, xr.SwapchainCreateInfo(
        usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT | xr.SwapchainUsageFlags.SAMPLED_BIT,
        format=fmt, sample_count=1, width=w, height=h, face_count=1, array_size=1, mip_count=1))
    images = xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)
    eye_fbos = []
    for img in images:
        fbo = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_TEXTURE_2D, img.image, 0)
        status = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
        if status != GL.GL_FRAMEBUFFER_COMPLETE:
            print("FBO incomplete:", status)
        eye_fbos.append(fbo)
    GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
    swapchains.append(sc); fbos.append(eye_fbos); sizes.append((w, h))
    print(f"swapchain {w}x{h} fmt={fmt} images={len(images)}")

# ---- reference space + controller pose actions ----
ref_space = xr.create_reference_space(session, xr.ReferenceSpaceCreateInfo(
    reference_space_type=xr.ReferenceSpaceType.LOCAL,
    pose_in_reference_space=xr.Posef(orientation=xr.Quaternionf(0, 0, 0, 1), position=xr.Vector3f(0, 0, 0))))

action_set = xr.create_action_set(instance, xr.ActionSetCreateInfo(
    action_set_name="so100", localized_action_set_name="so100", priority=0))
hand_paths = (xr.Path * 2)(xr.string_to_path(instance, "/user/hand/left"),
                           xr.string_to_path(instance, "/user/hand/right"))
pose_action = xr.create_action(action_set, xr.ActionCreateInfo(
    action_type=xr.ActionType.POSE_INPUT, action_name="grip_pose", localized_action_name="Grip pose",
    count_subaction_paths=2, subaction_paths=hand_paths))
trig_action = xr.create_action(action_set, xr.ActionCreateInfo(
    action_type=xr.ActionType.FLOAT_INPUT, action_name="trigger", localized_action_name="Trigger",
    count_subaction_paths=2, subaction_paths=hand_paths))
for profile in ("/interaction_profiles/oculus/touch_controller", "/interaction_profiles/khr/simple_controller"):
    b = [xr.ActionSuggestedBinding(pose_action, xr.string_to_path(instance, "/user/hand/left/input/grip/pose")),
         xr.ActionSuggestedBinding(pose_action, xr.string_to_path(instance, "/user/hand/right/input/grip/pose"))]
    if "oculus" in profile:
        b += [xr.ActionSuggestedBinding(trig_action, xr.string_to_path(instance, "/user/hand/left/input/trigger/value")),
              xr.ActionSuggestedBinding(trig_action, xr.string_to_path(instance, "/user/hand/right/input/trigger/value"))]
    else:
        b += [xr.ActionSuggestedBinding(trig_action, xr.string_to_path(instance, "/user/hand/left/input/select/click")),
              xr.ActionSuggestedBinding(trig_action, xr.string_to_path(instance, "/user/hand/right/input/select/click"))]
    arr = (xr.ActionSuggestedBinding * len(b))(*b)
    xr.suggest_interaction_profile_bindings(instance, xr.InteractionProfileSuggestedBinding(
        interaction_profile=xr.string_to_path(instance, profile), count_suggested_bindings=len(b), suggested_bindings=arr))
xr.attach_session_action_sets(session, xr.SessionActionSetsAttachInfo(count_action_sets=1,
                                                                      action_sets=ctypes.pointer(action_set)))
hand_spaces = [xr.create_action_space(session, xr.ActionSpaceCreateInfo(action=pose_action, subaction_path=p))
               for p in hand_paths]

# ---- event-driven session lifecycle ----
state = xr.SessionState.UNKNOWN
running = False
quit_requested = False


def poll_xr_events():
    global state, running, quit_requested
    while True:
        try:
            ev = xr.poll_event(instance)
        except xr.exception.EventUnavailable:
            return
        if ev.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
            sc = ctypes.cast(ctypes.byref(ev), ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
            state = xr.SessionState(sc.state)
            print("session state:", state.name)
            if state == xr.SessionState.READY:
                xr.begin_session(session, xr.SessionBeginInfo(primary_view_configuration_type=view_cfg))
                running = True
            elif state == xr.SessionState.STOPPING:
                xr.end_session(session); running = False
            elif state in (xr.SessionState.EXITING, xr.SessionState.LOSS_PENDING):
                running = False; quit_requested = True
        elif ev.type == xr.StructureType.EVENT_DATA_INTERACTION_PROFILE_CHANGED:
            print("interaction profile changed")


t0 = time.time(); frames = 0; last_report = 0.0
print(f"waiting for headset (put it on) ... {DURATION:.0f}s budget")
while not quit_requested and time.time() - t0 < DURATION:
    glfw.poll_events()
    poll_xr_events()
    if not running:
        time.sleep(0.01)
        continue
    fs = xr.wait_frame(session, xr.FrameWaitInfo())
    xr.begin_frame(session, xr.FrameBeginInfo())
    try:
        xr.sync_actions(session, xr.ActionsSyncInfo(count_active_action_sets=1,
                                                    active_action_sets=ctypes.pointer(xr.ActiveActionSet(action_set, xr.NULL_PATH))))
    except xr.exception.SessionNotFocused:
        pass  # XR_SESSION_NOT_FOCUSED is a success code; pyopenxr raises it
    proj_views = []
    if fs.should_render:
        view_state, views = xr.locate_views(session, xr.ViewLocateInfo(
            view_configuration_type=view_cfg, display_time=fs.predicted_display_time, space=ref_space))
        q = views[0].pose.orientation
        yaw = math.atan2(2 * (q.w * q.y + q.x * q.z), 1 - 2 * (q.y * q.y + q.z * q.z))
        for i, (v, sc, eye_fbos, (w, h)) in enumerate(zip(views, swapchains, fbos, sizes)):
            idx = xr.acquire_swapchain_image(sc, xr.SwapchainImageAcquireInfo())
            xr.wait_swapchain_image(sc, xr.SwapchainImageWaitInfo(timeout=xr.INFINITE_DURATION))
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, eye_fbos[idx])
            GL.glViewport(0, 0, w, h)
            # colour follows yaw so you can see tracking works; left eye slightly brighter
            GL.glClearColor(0.5 + 0.5 * math.sin(yaw), 0.5 + 0.5 * math.cos(yaw), 0.3 + 0.1 * i, 1.0)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT)
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
            xr.release_swapchain_image(sc, xr.SwapchainImageReleaseInfo())
            proj_views.append(xr.CompositionLayerProjectionView(
                pose=v.pose, fov=v.fov,
                sub_image=xr.SwapchainSubImage(swapchain=sc, image_rect=xr.Rect2Di(
                    offset=xr.Offset2Di(0, 0), extent=xr.Extent2Di(w, h)), image_array_index=0)))
        frames += 1
        if time.time() - last_report > 1.0:
            last_report = time.time()
            p = views[0].pose.position
            vflags = int(view_state.view_state_flags)
            line = f"t={time.time()-t0:4.1f}s f={frames:4d} state={state.name} viewflags={vflags:#x} head=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f}) yaw={math.degrees(yaw):+.0f}"
            for name, hs, hp in zip("LR", hand_spaces, hand_paths):
                loc = xr.locate_space(hs, ref_space, fs.predicted_display_time)
                ok = bool(loc.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT)
                tr = xr.get_action_state_float(session, xr.ActionStateGetInfo(action=trig_action, subaction_path=hp))
                hp_ = loc.pose.position
                line += f" {name}:{'ok' if ok else '--'}({hp_.x:+.2f},{hp_.y:+.2f},{hp_.z:+.2f}) trig={tr.current_state:.2f}"
            print(line, flush=True)
    layers = []
    if proj_views:
        pv_arr = (xr.CompositionLayerProjectionView * len(proj_views))(*proj_views)
        layer = xr.CompositionLayerProjection(space=ref_space, view_count=len(proj_views), views=pv_arr)
        layers = [ctypes.byref(layer)]
    xr.end_frame(session, xr.FrameEndInfo(display_time=fs.predicted_display_time,
                                          environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE, layers=layers))

print(f"done: {frames} frames in {time.time()-t0:.1f}s, final state {state.name}")
if running:
    xr.request_exit_session(session)
    for _ in range(200):
        poll_xr_events()
        if not running:
            break
        time.sleep(0.01)
xr.destroy_session(session)
xr.destroy_instance(instance)
glfw.terminate()
print("STAGE2 OK" if frames > 0 else "STAGE2 NO FRAMES (headset never became READY/FOCUSED?)")
