"""pyopenxr smoke test against WiVRn.
Stage 1 (default): create instance, print runtime + system.
Stage 2 (--render): run a ContextObject session for ~8 s, print head and controller poses.
"""
import sys
import time

import xr


def s(x):
    return x.decode() if isinstance(x, bytes) else x


exts = [s(e.extension_name) for e in xr.enumerate_instance_extension_properties()]
print("runtime extensions:", len(exts))
print("  interesting:", [e for e in exts if "hand" in e.lower() or "opengl" in e.lower() or "vulkan" in e.lower()])

if "--render" not in sys.argv:
    inst = xr.create_instance(xr.InstanceCreateInfo(
        application_info=xr.ApplicationInfo(
            application_name="so100_hello", application_version=1,
            engine_name="none", engine_version=0,
            api_version=xr.XR_CURRENT_API_VERSION)))
    props = xr.get_instance_properties(inst)
    print("runtime:", s(props.runtime_name), "version", props.runtime_version)
    sysid = xr.get_system(inst, xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
    sp = xr.get_system_properties(inst, sysid)
    gp = sp.graphics_properties
    print("system:", s(sp.system_name), "max swapchain",
          gp.max_swapchain_image_width, "x", gp.max_swapchain_image_height)
    for cfg in xr.enumerate_view_configurations(inst, sysid):
        views = xr.enumerate_view_configuration_views(inst, sysid, xr.ViewConfigurationType(cfg))
        print("view config", cfg, [(v.recommended_image_rect_width, v.recommended_image_rect_height) for v in views])
    xr.destroy_instance(inst)
    print("STAGE1 OK")
    sys.exit(0)

# ---------------- Stage 2: real session with OpenGL ----------------
with xr.ContextObject(
    instance_create_info=xr.InstanceCreateInfo(
        enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]),
) as ctx:
    print("session created; runtime", s(xr.get_instance_properties(ctx.instance).runtime_name))
    ctrl_paths = (xr.Path * 2)(
        xr.string_to_path(ctx.instance, "/user/hand/left"),
        xr.string_to_path(ctx.instance, "/user/hand/right"))
    pose_action = xr.create_action(ctx.default_action_set, xr.ActionCreateInfo(
        action_type=xr.ActionType.POSE_INPUT, action_name="hand_pose",
        localized_action_name="Hand Pose", count_subaction_paths=2, subaction_paths=ctrl_paths))
    for profile in ("/interaction_profiles/oculus/touch_controller",
                    "/interaction_profiles/khr/simple_controller"):
        bindings = (xr.ActionSuggestedBinding * 2)(
            xr.ActionSuggestedBinding(pose_action, xr.string_to_path(ctx.instance, "/user/hand/left/input/grip/pose")),
            xr.ActionSuggestedBinding(pose_action, xr.string_to_path(ctx.instance, "/user/hand/right/input/grip/pose")))
        try:
            xr.suggest_interaction_profile_bindings(ctx.instance, xr.InteractionProfileSuggestedBinding(
                interaction_profile=xr.string_to_path(ctx.instance, profile),
                count_suggested_bindings=2, suggested_bindings=bindings))
        except xr.XrException as e:
            print("binding", profile, "->", e)
    spaces = [xr.create_action_space(ctx.session, xr.ActionSpaceCreateInfo(action=pose_action, subaction_path=p))
              for p in ctrl_paths]
    t0 = time.time()
    n = 0
    last = 0.0
    for frame_state in ctx.frame_loop():
        n += 1
        report = time.time() - last > 1.0
        for i, v in enumerate(ctx.view_loop(frame_state)):
            if report and i == 0:
                p = v.pose.position
                print(f"t={time.time()-t0:4.1f}s frame {n} state={ctx.session_state.name} "
                      f"head=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})")
        if report:
            last = time.time()
            for name, sp in zip(("L", "R"), spaces):
                loc = xr.locate_space(sp, ctx.space, frame_state.predicted_display_time)
                ok = bool(loc.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT)
                p = loc.pose.position
                print(f"   {name}: valid={ok} ({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})")
        if time.time() - t0 > 8:
            break
    print("STAGE2 OK frames", n)
