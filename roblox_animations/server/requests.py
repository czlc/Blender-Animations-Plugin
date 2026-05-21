"""
Request processing and task management for the animation server.
"""

import json
import traceback
import zlib
import bpy
import bpy_extras
from mathutils import Matrix
from ..core.utils import (
    get_action_hash,
    get_scene_fps,
    set_scene_fps,
    cf_to_mat,
    get_object_by_name,
    iter_scene_objects,
)
from ..core import utils
from ..animation.serialization import serialize, is_deform_bone_rig
from ..animation.import_export import import_animation_preserve_ik


# Global request queues
pending_requests = []
pending_responses = {}

transform_to_blender = bpy_extras.io_utils.axis_conversion(
    from_forward="Z", from_up="Y", to_forward="-Y", to_up="Z"
).to_4x4()  # transformation matrix from Y-up to Z-up


def _get_reported_bone_parent_name(bone):
    bone_data = getattr(bone, "bone", bone)
    original_parent = bone_data.get("rbx_original_parent", "") if bone_data is not None else ""
    if isinstance(original_parent, str) and original_parent:
        return original_parent

    parent = getattr(bone, "parent", None)
    if parent is None:
        return None

    parent_name = getattr(parent, "name", None)
    if isinstance(parent_name, str) and parent_name:
        return parent_name

    parent_bone = getattr(parent, "bone", None)
    parent_bone_name = getattr(parent_bone, "name", None)
    if isinstance(parent_bone_name, str) and parent_bone_name:
        return parent_bone_name

    return None


MRXEN0_R15_RIG_ID = "xqpb4vnl0zay"
MRXEN0_IMPORT_DEBUG_REV = "fk_head_control_probe_20260521"

MRXEN0_R15_FK_BONE_CANDIDATES = {
    "LowerTorso": ("LowTorso", "LowerTorso", "TORSO", "BODY", "ROOT"),
    "UpperTorso": ("UpTorso", "UpperTorso", "TORSO", "BODY", "Chest"),
    "Head": ("FK_Head", "HEAD", "Head"),
    "LeftUpperArm": ("FK_UpperArm.L", "LeftUpperArm"),
    "LeftLowerArm": ("FK_LowerArm.L", "LeftLowerArm"),
    "LeftHand": ("FK_Hand.L", "LeftHand"),
    "RightUpperArm": ("FK_UpperArm.R", "RightUpperArm"),
    "RightLowerArm": ("FK_LowerArm.R", "RightLowerArm"),
    "RightHand": ("FK_Hand.R", "RightHand"),
    "LeftUpperLeg": ("FK_UpperLeg.L", "LeftUpperLeg"),
    "LeftLowerLeg": ("FK_LowerLeg.L", "LeftLowerLeg"),
    "LeftFoot": ("FK_Foot.L", "LeftFoot"),
    "RightUpperLeg": ("FK_UpperLeg.R", "RightUpperLeg"),
    "RightLowerLeg": ("FK_LowerLeg.R", "RightLowerLeg"),
    "RightFoot": ("FK_Foot.R", "RightFoot"),
}

MRXEN0_R15_FK_PROPS = (
    "ARM_IK_FK.L",
    "ARM_IK_FK.R",
    "LEG_IK_FK.L",
    "LEG_IK_FK.R",
)

MRXEN0_R15_IMPORT_PROPS = (
    ("HEAD_FOLLOW", 1.0),
)

MRXEN0_R15_PARENT = {
    "UpperTorso": "LowerTorso",
    "Head": "UpperTorso",
    "LeftUpperArm": "UpperTorso",
    "LeftLowerArm": "LeftUpperArm",
    "LeftHand": "LeftLowerArm",
    "RightUpperArm": "UpperTorso",
    "RightLowerArm": "RightUpperArm",
    "RightHand": "RightLowerArm",
    "LeftUpperLeg": "LowerTorso",
    "LeftLowerLeg": "LeftUpperLeg",
    "LeftFoot": "LeftLowerLeg",
    "RightUpperLeg": "LowerTorso",
    "RightLowerLeg": "RightUpperLeg",
    "RightFoot": "RightLowerLeg",
}

MRXEN0_R15_LIMB_SOURCES = {
    "LeftUpperArm",
    "LeftLowerArm",
    "LeftHand",
    "RightUpperArm",
    "RightLowerArm",
    "RightHand",
    "LeftUpperLeg",
    "LeftLowerLeg",
    "LeftFoot",
    "RightUpperLeg",
    "RightLowerLeg",
    "RightFoot",
}

def _is_mrxen0_r15_rig(ao):
    if not ao or ao.type != "ARMATURE" or not getattr(ao, "data", None):
        return False
    if ao.data.get("rig_id") == MRXEN0_R15_RIG_ID:
        return True
    bone_names = {bone.name for bone in ao.data.bones}
    required = {
        "PROPERTIES",
        "FK_UpperArm.L",
        "FK_LowerArm.L",
        "FK_Hand.L",
        "FK_UpperArm.R",
        "FK_LowerArm.R",
        "FK_Hand.R",
        "FK_UpperLeg.L",
        "FK_LowerLeg.L",
        "FK_Foot.L",
        "FK_UpperLeg.R",
        "FK_LowerLeg.R",
        "FK_Foot.R",
    }
    return required.issubset(bone_names)


def _mrxen0_fk_target_name(ao, roblox_bone_name):
    for candidate in MRXEN0_R15_FK_BONE_CANDIDATES.get(roblox_bone_name, ()):
        if candidate in ao.pose.bones:
            return candidate
    return None


def _mrxen0_body_like_bone_names(ao):
    tokens = (
        "torso",
        "body",
        "root",
        "hip",
        "hips",
        "chest",
        "spine",
        "waist",
        "pelvis",
        "cog",
        "com",
    )
    return sorted(
        bone.name
        for bone in ao.pose.bones
        if any(token in bone.name.lower() for token in tokens)
    )


def _mrxen0_head_like_bone_names(ao):
    tokens = ("head", "neck")
    return sorted(
        bone.name
        for bone in ao.pose.bones
        if any(token in bone.name.lower() for token in tokens)
    )


def _mrxen0_source_depth(source_name):
    depth = 0
    parent = MRXEN0_R15_PARENT.get(source_name)
    seen = set()
    while parent and parent not in seen:
        seen.add(parent)
        depth += 1
        parent = MRXEN0_R15_PARENT.get(parent)
    return depth


def _mrxen0_matrix_error(expected_matrix, actual_matrix):
    delta = expected_matrix @ actual_matrix.inverted()
    loc_error = delta.to_translation().length
    rot_error_deg = delta.to_quaternion().angle * 57.29577951308232
    return loc_error, rot_error_deg


def _pose_local_rest_matrix(pose_bone):
    if pose_bone.parent:
        return pose_bone.parent.bone.matrix_local.inverted() @ pose_bone.bone.matrix_local
    return pose_bone.bone.matrix_local.copy()


def _apply_local_delta_to_pose_bone(pose_bone, local_delta):
    local_rest = _pose_local_rest_matrix(pose_bone)
    if pose_bone.parent:
        pose_bone.matrix = pose_bone.parent.matrix @ (local_rest @ local_delta)
    else:
        pose_bone.matrix = local_rest @ local_delta


def _copy_rotation_with_translation(rotation_matrix, translation_matrix):
    matrix = rotation_matrix.to_3x3().to_4x4()
    matrix.translation = translation_matrix.to_translation()
    return matrix


def _mrxen0_virtual_source_rest_matrix(ao, source_bone_name):
    source_bone = ao.pose.bones.get(source_bone_name)
    if not source_bone:
        return None

    actual_rest = source_bone.bone.matrix_local.copy()
    if source_bone_name not in MRXEN0_R15_LIMB_SOURCES:
        return actual_rest

    if "Arm" in source_bone_name or "Hand" in source_bone_name:
        frame_source_name = "UpperTorso"
    else:
        frame_source_name = "LowerTorso"

    frame_source_bone = ao.pose.bones.get(frame_source_name)
    if not frame_source_bone:
        return actual_rest

    # Roblox KeyFrameSequence pose CFrames are authored in body-part space.
    # MrXen0's limb bones are oriented along the drawn bone chain, so use the
    # torso part frame rotation plus each limb bone's rest position as a virtual
    # Roblox part frame before mapping back to the rig's actual FK controls.
    return _copy_rotation_with_translation(
        frame_source_bone.bone.matrix_local,
        actual_rest,
    )


def _apply_source_delta_to_mrxen0_control(
    ao,
    source_bone_name,
    target_bone_name,
    local_delta,
    source_pose_matrices,
):
    source_bone = ao.pose.bones.get(source_bone_name)
    target_bone = ao.pose.bones.get(target_bone_name)
    if not target_bone:
        return None

    actual_source_rest = (
        source_bone.bone.matrix_local.copy()
        if source_bone
        else target_bone.bone.matrix_local.copy()
    )
    source_rest = (
        _mrxen0_virtual_source_rest_matrix(ao, source_bone_name)
        or actual_source_rest.copy()
    )
    target_rest = target_bone.bone.matrix_local.copy()

    # MrXen0 FK controls do not share the same local space as Roblox Pose bones.
    # First reconstruct the desired Roblox source bone object-space pose through
    # the source hierarchy, then carry over the rest-pose offset to the FK
    # control, mirroring the rig's snap operators.
    parent_source_name = MRXEN0_R15_PARENT.get(source_bone_name)
    parent_source_bone = (
        ao.pose.bones.get(parent_source_name) if parent_source_name else None
    )
    parent_pose_matrix = source_pose_matrices.get(parent_source_name)
    parent_source_rest = (
        _mrxen0_virtual_source_rest_matrix(ao, parent_source_name)
        if parent_source_name
        else None
    )

    if parent_source_bone and parent_pose_matrix is not None and parent_source_rest:
        local_rest = parent_source_rest.inverted() @ source_rest
        source_pose_matrix = parent_pose_matrix @ local_rest @ local_delta
    else:
        source_pose_matrix = source_rest @ local_delta

    source_pose_matrices[source_bone_name] = source_pose_matrix
    desired_source_matrix = (
        source_pose_matrix @ source_rest.inverted() @ actual_source_rest
    )
    target_bone.matrix = (
        desired_source_matrix @ actual_source_rest.inverted() @ target_rest
    )

    if source_bone and source_bone is not target_bone:
        for _ in range(3):
            bpy.context.view_layer.update()
            actual_source_matrix = source_bone.matrix.copy()
            try:
                correction = desired_source_matrix @ actual_source_matrix.inverted()
            except Exception:
                break

            loc_error = correction.to_translation().length
            rot_error = correction.to_quaternion().angle
            if loc_error < 0.0001 and rot_error < 0.0001:
                break

            target_bone.matrix = correction @ target_bone.matrix

    return desired_source_matrix


def _set_mrxen0_fk_mode(ao, frame=None, value=None):
    properties = ao.pose.bones.get("PROPERTIES")
    if not properties:
        return []

    if value is None:
        value = 0.0

    changed = []
    for prop_name in MRXEN0_R15_FK_PROPS:
        if prop_name in properties:
            properties[prop_name] = float(value)
            changed.append(f"{prop_name}={float(value):.0f}")
            if frame is not None:
                try:
                    properties.keyframe_insert(data_path=f'["{prop_name}"]', frame=frame)
                except Exception:
                    pass

    for prop_name, prop_value in MRXEN0_R15_IMPORT_PROPS:
        if prop_name in properties:
            properties[prop_name] = float(prop_value)
            changed.append(f"{prop_name}={float(prop_value):.0f}")
            if frame is not None:
                try:
                    properties.keyframe_insert(data_path=f'["{prop_name}"]', frame=frame)
                except Exception:
                    pass

    bpy.context.view_layer.update()
    return changed


def _extract_pose_cframe(pose_data):
    easing_style = "Linear"
    easing_direction = "In"

    if isinstance(pose_data, list) and len(pose_data) == 3:
        cframe_components = pose_data[0]
        easing_style = pose_data[1]
        easing_direction = pose_data[2]
    elif isinstance(pose_data, dict):
        cframe_components = pose_data.get("components", [])
        easing_style = pose_data.get("easingStyle", "Linear")
        easing_direction = pose_data.get("easingDirection", "In")
    elif isinstance(pose_data, list):
        cframe_components = pose_data
    else:
        cframe_components = []

    return cframe_components, easing_style, easing_direction


def process_pending_requests():
    """Process any pending animation requests"""
    try:
        if pending_requests:  # Check if list is not empty
            print(f"Blender Addon: Processing {len(pending_requests)} pending requests")
            request = pending_requests.pop(0)
            request_type = request[0]

            if request_type == "export_animation":
                _, task_id, armature_name = request
                print(
                    f"Blender Addon: Dispatching export_animation task (task_id={task_id}, armature={armature_name})"
                )
                execute_in_main_thread(task_id, armature_name)
            elif request_type == "list_armatures":
                _, task_id = request
                print(
                    f"Blender Addon: Dispatching list_armatures task (task_id={task_id})"
                )
                execute_list_armatures(task_id)
            elif request_type == "import":
                if len(request) == 4:
                    _, task_id, animation_data, target_armature = request
                    print(
                        f"Blender Addon: Dispatching import task (task_id={task_id}, target={target_armature})"
                    )
                    execute_import_animation(task_id, animation_data, target_armature)
                else:
                    _, task_id, animation_data = request
                    print(f"Blender Addon: Dispatching import task (task_id={task_id})")
                    execute_import_animation(task_id, animation_data)
            elif request_type == "get_bone_rest":
                _, task_id, armature_name = request
                print(
                    f"Blender Addon: Dispatching get_bone_rest task (task_id={task_id}, armature={armature_name})"
                )
                execute_get_bone_rest(task_id, armature_name)

    except Exception as e:
        print(f"Blender Addon: Error processing requests: {str(e)}")
        traceback.print_exc()
    return 0.01  # Run every 10ms for good balance


def execute_list_armatures(task_id):
    """Execute the armature listing in the main thread"""
    try:
        print("Blender Addon: Executing list_armatures in main thread...")

        # Force refresh by invalidating cache first
        utils.invalidate_armature_cache()

        fresh_armatures = [
            obj.name for obj in iter_scene_objects() if obj.type == "ARMATURE"
        ]

        armatures = []
        for armature_name in fresh_armatures:
            obj = get_object_by_name(armature_name)

            if obj:  # Double-check object still exists
                # build bone hierarchy map: { bone_name: parent_bone_name or None }
                bone_hierarchy = {}
                for bone in obj.data.bones:
                    bone_hierarchy[bone.name] = _get_reported_bone_parent_name(bone)

                armature_info = {
                    "name": obj.name,
                    "bones": [bone.name for bone in obj.data.bones],
                    "num_bones": len(obj.data.bones),
                    "has_animation": bool(
                        obj.animation_data and obj.animation_data.action
                    ),
                    "frame_range": [
                        bpy.context.scene.frame_start,
                        bpy.context.scene.frame_end,
                    ]
                    if obj.animation_data
                    else None,
                    "bone_hierarchy": bone_hierarchy,
                }
                armatures.append(armature_info)

                # Pre-cache the hash for this armature
                action = obj.animation_data.action if obj.animation_data else None
                utils.armature_anim_hashes[obj.name] = get_action_hash(action)

        found_armature_names = [a["name"] for a in armatures]
        print(f"Blender Addon: Found armatures: {found_armature_names}")

        response = {
            "armatures": armatures,
            "current": getattr(
                getattr(bpy.context.scene, "rbx_anim_settings", None),
                "rbx_anim_armature",
                None,
            ),
            "fps": bpy.context.scene.render.fps,
        }

        data = json.dumps(response).encode("utf-8")
        print(
            f"Blender Addon: Prepared response for task {task_id}. Armature count: {len(armatures)}"
        )
        pending_responses[task_id] = (True, data)

    except Exception as e:
        print(f"Blender Addon: Error in execute_list_armatures: {str(e)}")
        traceback.print_exc()
        pending_responses[task_id] = (
            False,
            f"Error listing armatures: {str(e)}",
        )


def execute_in_main_thread(task_id, armature_name):
    """Execute the animation export in the main thread"""
    try:
        if not armature_name:
            pending_responses[task_id] = (False, "No armature selected")
            return

        ao = get_object_by_name(armature_name)
        if not ao:
            pending_responses[task_id] = (
                False,
                f"Armature '{armature_name}' not found.",
            )
            return

        if ao.type != "ARMATURE":
            pending_responses[task_id] = (
                False,
                f"Object '{armature_name}' is not an armature (type: {ao.type}). Please select a valid armature object.",
            )
            return

        bpy.context.view_layer.objects.active = ao
        # Only switch mode if necessary to avoid expensive operations
        if ao.mode != "POSE":
            print(f"Blender Addon: Switching to POSE mode for '{armature_name}'...")
            bpy.ops.object.mode_set(mode="POSE")

        desired_fps = get_scene_fps()
        set_scene_fps(desired_fps)

        # Check if this is a deform bone rig or if deform bone serialization is forced
        settings = getattr(bpy.context.scene, "rbx_anim_settings", None)
        force_deform = getattr(settings, "force_deform_bone_serialization", False)

        use_deform_bone_serialization = is_deform_bone_rig(ao) or force_deform
        print(
            f"Server export: Using {'deform bone' if use_deform_bone_serialization else 'Motor6D'} serialization"
        )
        print(f"Blender Addon: Starting animation export for '{armature_name}'...")

        serialized = serialize(ao)
        if not serialized:
            pending_responses[task_id] = (
                False,
                f"Failed to serialize animation for '{armature_name}'. Check if the armature has animation data or keyframes.",
            )
            return

        if not serialized.get("kfs") or len(serialized["kfs"]) == 0:
            pending_responses[task_id] = (
                False,
                f"No animation data found for '{armature_name}'. Please add keyframes or animation data to the armature.",
            )
            return

        encoded = json.dumps(serialized, separators=(",", ":"))
        compressed = zlib.compress(encoded.encode("utf-8"))

        pending_responses[task_id] = (True, compressed)

    except Exception as e:
        print(f"Error in main thread: {str(e)}")
        traceback.print_exc()
        pending_responses[task_id] = (False, f"Error during export: {str(e)}")


def execute_import_animation(task_id, animation_data, target_armature=None):
    """Execute the animation import in the main thread with IK preservation."""
    try:
        # Use target armature if provided, otherwise fall back to scene selection
        if target_armature:
            armature_name = target_armature
        else:
            settings = getattr(bpy.context.scene, "rbx_anim_settings", None)
            armature_name = settings.rbx_anim_armature if settings else None

        if not armature_name:
            raise ValueError(
                "No armature specified for import. Please provide target armature or select one in the scene."
            )

        ao = get_object_by_name(armature_name)
        if not ao:
            raise ValueError(
                f"Selected object '{armature_name}' is not a valid armature."
            )

        if ao.type != "ARMATURE":
            raise ValueError(
                f"Selected object '{armature_name}' is not a valid armature."
            )

        bpy.context.view_layer.objects.active = ao
        if ao.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")

        use_mrxen0_fk_import = _is_mrxen0_r15_rig(ao)

        def _do_import():
            # Clear existing action data. Do not clear animation_data on rigs that
            # depend on drivers for FK/IK constraints; animation_data_clear()
            # removes those drivers and leaves controls moving without the model.
            if ao.animation_data and use_mrxen0_fk_import:
                ao.animation_data.action = None
            elif ao.animation_data:
                ao.animation_data_clear()

            # Reset pose to rest position to ensure a clean slate
            if ao.pose:
                for bone in ao.pose.bones:
                    bone.matrix_basis = Matrix.Identity(4)
                bpy.context.view_layer.update()  # Ensure the pose update is registered

            action = bpy.data.actions.new(name=f"{armature_name}_ImportedAnimation")
            ao.animation_data_create()
            ao.animation_data.action = action

            # Ensure a compatible action slot exists (Blender 4.4+)
            active_slot = None
            if hasattr(action, "slots"):
                if action.slots:
                    active_slot = action.slots[0]
                else:
                    try:
                        active_slot = action.slots.new(
                            id_type="OBJECT", name=f"OB{ao.name}"
                        )
                    except TypeError:
                        active_slot = action.slots.new(id_type="OBJECT")
                if active_slot and hasattr(ao.animation_data, "action_slot"):
                    try:
                        ao.animation_data.action_slot = active_slot
                    except Exception:
                        pass
            else:
                pass

            fps = animation_data.get("export_info", {}).get("fps", get_scene_fps())
            set_scene_fps(fps)

            scene = bpy.context.scene
            scene.frame_start = 0
            scene.frame_end = int(animation_data["t"] * fps)

            is_deform_rig = animation_data.get("is_deform_bone_rig", False)
            incoming_bone_names = {
                bone_name
                for kf_data in animation_data.get("kfs", [])
                for bone_name in (kf_data.get("kf") or {}).keys()
            }
            mrxen0_source_to_target = {}
            mrxen0_unmapped_sources = set()
            if use_mrxen0_fk_import:
                for source_name in incoming_bone_names:
                    target_name = _mrxen0_fk_target_name(ao, source_name)
                    if target_name:
                        mrxen0_source_to_target[source_name] = target_name
                    else:
                        mrxen0_unmapped_sources.add(source_name)

                print(
                    "Blender Addon: MrXen0 R15 rig detected; importing Roblox R15 "
                    "motion to FK controls. "
                    f"rev={MRXEN0_IMPORT_DEBUG_REV}"
                )
                print(
                    "Blender Addon: MrXen0 FK map "
                    f"{len(mrxen0_source_to_target)}/{len(incoming_bone_names)} source bones."
                )
                body_like_bones = _mrxen0_body_like_bone_names(ao)
                if body_like_bones:
                    print(
                        "Blender Addon: MrXen0 body-like bones: "
                        + ", ".join(body_like_bones[:80])
                    )
                    if len(body_like_bones) > 80:
                        print(
                            "Blender Addon: MrXen0 body-like bones truncated, "
                            f"total={len(body_like_bones)}"
                        )
                head_like_bones = _mrxen0_head_like_bone_names(ao)
                if head_like_bones:
                    print(
                        "Blender Addon: MrXen0 head-like bones: "
                        + ", ".join(head_like_bones[:80])
                    )
                if mrxen0_unmapped_sources:
                    print(
                        "Blender Addon: MrXen0 unmapped source bones: "
                        + ", ".join(sorted(mrxen0_unmapped_sources))
                    )
                for source_name in sorted(mrxen0_source_to_target):
                    print(
                        "Blender Addon: MrXen0 map "
                        f"{source_name} -> FK {mrxen0_source_to_target[source_name]}"
                    )
                lower_target = mrxen0_source_to_target.get("LowerTorso")
                upper_target = mrxen0_source_to_target.get("UpperTorso")
                if lower_target and upper_target and lower_target == upper_target:
                    print(
                        "Blender Addon: MrXen0 WARNING LowerTorso and UpperTorso map "
                        f"to the same control '{lower_target}'. Torso motion may be approximate."
                    )

            # This will store all transform data for all bones across all frames before we create any keyframes.
            # Format: { bone_name: { frame: {"location": Vector, "rotation": Quaternion, "scale": Vector}, ... }, ... }
            all_bone_data = {}
            mrxen0_verify_sample_frames = set()
            mrxen0_verify_errors = []
            if use_mrxen0_fk_import:
                imported_frames = sorted(
                    {
                        int(kf_data["t"] * fps)
                        for kf_data in animation_data.get("kfs", [])
                    }
                )
                if imported_frames:
                    mrxen0_verify_sample_frames.add(imported_frames[0])
                    mrxen0_verify_sample_frames.add(imported_frames[len(imported_frames) // 2])
                    mrxen0_verify_sample_frames.add(imported_frames[-1])

            # Reset pose to rest position and pre-populate all transformable bones
            # with their rest pose at the start frame. This ensures a defined initial state.
            if ao.pose:
                for bone in ao.pose.bones:
                    bone.matrix_basis = Matrix.Identity(4)
                bpy.context.view_layer.update()

                start_frame = scene.frame_start
                if use_mrxen0_fk_import:
                    fk_mode_value = 0.0
                    fk_props = _set_mrxen0_fk_mode(ao, start_frame, fk_mode_value)
                    if fk_props:
                        print(
                            "Blender Addon: MrXen0 import rig properties: "
                            + ", ".join(fk_props)
                        )

                for bone in ao.pose.bones:
                    if use_mrxen0_fk_import:
                        is_transformable = bone.name in set(mrxen0_source_to_target.values())
                    elif is_deform_rig:
                        is_transformable = bone.bone.use_deform
                    else:
                        # Handle both boolean True and integer 1 for backward compatibility
                        # We check truthiness to support both True (bool) and 1 (int)
                        is_transformable = bool(bone.bone.get("is_transformable", False))
                    if is_transformable:
                        if use_mrxen0_fk_import:
                            bone.rotation_mode = "QUATERNION"
                        all_bone_data[bone.name] = {
                            start_frame: {
                                "location": bone.location.copy(),
                                "rotation_quaternion": bone.rotation_quaternion.copy(),
                                "scale": bone.scale.copy(),
                            }
                        }

            for kf_data in animation_data["kfs"]:
                frame = int(kf_data["t"] * fps)
                state = kf_data["kf"]
                mrxen0_source_pose_matrices = {}
                mrxen0_expected_source_matrices = {}

                bones_to_process = []
                for source_bone_name in state.keys():
                    if use_mrxen0_fk_import:
                        target_bone_name = mrxen0_source_to_target.get(source_bone_name)
                        pose_bone = ao.pose.bones.get(target_bone_name) if target_bone_name else None
                    else:
                        target_bone_name = source_bone_name
                        pose_bone = ao.pose.bones.get(source_bone_name)
                    if pose_bone:
                        bones_to_process.append((pose_bone, source_bone_name, target_bone_name))

                if use_mrxen0_fk_import:
                    bones_to_process.sort(key=lambda item: _mrxen0_source_depth(item[1]))
                else:
                    bones_to_process.sort(key=lambda item: len(item[0].parent_recursive))

                # Simplified single-pass processing loop.
                # By iterating through bones sorted by hierarchy (parents first), we ensure
                # that when we calculate a child's matrix, the parent's matrix for the
                # current frame has already been set.
                for pose_bone, source_bone_name, bone_name in bones_to_process:
                    bone_name = pose_bone.name
                    pose_data = state.get(source_bone_name)
                    if not pose_data:
                        continue

                    # Backwards compatibility: Handle old list-based format and new dict-based format.
                    cframe_components, easing_style, easing_direction = _extract_pose_cframe(pose_data)

                    if not cframe_components:
                        continue

                    bone_transform = cf_to_mat(cframe_components)

                    if use_mrxen0_fk_import:
                        expected_source_matrix = _apply_source_delta_to_mrxen0_control(
                            ao,
                            source_bone_name,
                            bone_name,
                            bone_transform,
                            mrxen0_source_pose_matrices,
                        )
                        if expected_source_matrix is not None:
                            mrxen0_expected_source_matrices[source_bone_name] = (
                                expected_source_matrix.copy()
                            )
                        bpy.context.view_layer.update()
                        final_matrix = None
                    else:
                        final_matrix = None

                    # --- Matrix Calculation ---
                    # Check if this bone has Motor6D properties (from rig build)
                    has_motor6d_props = (
                        "nicetransform" in pose_bone.bone
                        and "transform" in pose_bone.bone
                        and "transform1" in pose_bone.bone
                    )

                    if use_mrxen0_fk_import:
                        pass
                    elif not has_motor6d_props:
                        # Simple delta path - works for deform bones and any bone without Motor6D data
                        # The transform is a LOCAL delta in the bone's own space.
                        # Just apply it directly to the rest pose.
                        final_matrix = pose_bone.bone.matrix_local @ bone_transform
                    else:  # Motor6D rig
                        back_trans = transform_to_blender.inverted()
                        extr_transform = Matrix(pose_bone.bone["nicetransform"]).inverted()

                        orig_mat = Matrix(pose_bone.bone["transform"])
                        orig_mat_tr1 = Matrix(pose_bone.bone["transform1"])

                        if pose_bone.parent and "transform" in pose_bone.parent.bone:
                            parent_orig_mat = Matrix(pose_bone.parent.bone["transform"])
                            parent_orig_mat_tr1 = Matrix(
                                pose_bone.parent.bone["transform1"]
                            )

                            orig_base_mat = back_trans @ (orig_mat @ orig_mat_tr1)
                            parent_orig_base_mat = back_trans @ (
                                parent_orig_mat @ parent_orig_mat_tr1
                            )
                            orig_transform = parent_orig_base_mat.inverted() @ orig_base_mat

                            cur_transform = orig_transform @ bone_transform

                            parent_extr_transform = Matrix(
                                pose_bone.parent.bone["nicetransform"]
                            ).inverted()

                            # Use the parent's current matrix from the pose, which was set in the previous iteration
                            parent_matrix = pose_bone.parent.matrix
                            parent_obj_transform = back_trans @ (
                                parent_matrix @ parent_extr_transform
                            )

                            cur_obj_transform = parent_obj_transform @ cur_transform
                        else:
                            cur_obj_transform = bone_transform

                        final_matrix = (
                            transform_to_blender
                            @ cur_obj_transform
                            @ extr_transform.inverted()
                        )

                    # --- Apply and Store ---
                    if final_matrix is not None:
                        pose_bone.matrix = final_matrix

                    if bone_name in all_bone_data:
                        all_bone_data[bone_name][frame] = {
                            "location": pose_bone.location.copy(),
                            "rotation_quaternion": pose_bone.rotation_quaternion.copy(),
                            "scale": pose_bone.scale.copy(),
                            "easingStyle": easing_style,
                            "easingDirection": easing_direction,
                        }

                if use_mrxen0_fk_import and frame in mrxen0_verify_sample_frames:
                    bpy.context.view_layer.update()
                    for source_name, expected_matrix in mrxen0_expected_source_matrices.items():
                        source_bone = ao.pose.bones.get(source_name)
                        if not source_bone:
                            continue
                        loc_error, rot_error_deg = _mrxen0_matrix_error(
                            expected_matrix,
                            source_bone.matrix,
                        )
                        mrxen0_verify_errors.append(
                            (frame, source_name, loc_error, rot_error_deg)
                        )

            if use_mrxen0_fk_import and mrxen0_verify_errors:
                max_loc_error = max(error[2] for error in mrxen0_verify_errors)
                max_rot_error = max(error[3] for error in mrxen0_verify_errors)
                avg_loc_error = sum(error[2] for error in mrxen0_verify_errors) / len(
                    mrxen0_verify_errors
                )
                avg_rot_error = sum(error[3] for error in mrxen0_verify_errors) / len(
                    mrxen0_verify_errors
                )
                print(
                    "Blender Addon: MrXen0 verification summary "
                    f"samples={len(mrxen0_verify_sample_frames)} "
                    f"comparisons={len(mrxen0_verify_errors)} "
                    f"max_loc={max_loc_error:.6f} avg_loc={avg_loc_error:.6f} "
                    f"max_rot_deg={max_rot_error:.4f} avg_rot_deg={avg_rot_error:.4f}"
                )
                for frame, source_name, loc_error, rot_error_deg in sorted(
                    mrxen0_verify_errors,
                    key=lambda error: (error[3], error[2]),
                    reverse=True,
                )[:8]:
                    print(
                        "Blender Addon: MrXen0 verify worst "
                        f"frame={frame} bone={source_name} "
                        f"loc={loc_error:.6f} rot_deg={rot_error_deg:.4f}"
                    )

            for bone_name, frame_data in all_bone_data.items():
                sorted_frames = sorted(frame_data.keys())

                channelbag = utils.get_action_channelbag(action)
                if channelbag is None or not hasattr(channelbag, "fcurves"):
                    legacy_fcurves = getattr(action, "fcurves", None)
                    if legacy_fcurves is None:
                        raise RuntimeError(
                            "Unable to access animation channelbag for import"
                        )

                    class _LegacyChannelbag:
                        def __init__(self, fcurves, groups):
                            self.fcurves = fcurves
                            self.groups = groups

                        def new(self, *args, **kwargs):
                            return self.fcurves.new(*args, **kwargs)

                    channelbag = _LegacyChannelbag(
                        legacy_fcurves, getattr(action, "groups", [])
                    )

                group_collections = []
                action_groups = getattr(action, "groups", None)
                if action_groups is not None:
                    group_collections.append(action_groups)
                channelbag_groups = getattr(channelbag, "groups", None)
                if channelbag_groups is not None and channelbag_groups is not action_groups:
                    group_collections.append(channelbag_groups)

                def ensure_group(group_name):
                    if not group_collections:
                        return None
                    for group_collection in group_collections:
                        if group_collection is None:
                            continue
                        existing = None
                        try:
                            existing = group_collection.get(group_name)
                        except Exception:
                            existing = None
                        if existing is not None:
                            return existing
                        try:
                            return group_collection.new(name=group_name)
                        except TypeError:
                            try:
                                return group_collection.new(group_name)
                            except Exception:
                                continue
                        except Exception:
                            continue
                    return None

                # Create fcurves with version-appropriate parameters
                def create_fcurve(data_path, index, group_name):
                    fcurve = None
                    try:
                        fcurve = channelbag.fcurves.new(data_path, index=index)
                    except TypeError:
                        if hasattr(channelbag.fcurves, "new"):
                            for candidate in ("group_name", "action_group"):
                                try:
                                    fcurve = channelbag.fcurves.new(
                                        data_path, index=index, **{candidate: group_name}
                                    )
                                    break
                                except TypeError:
                                    continue
                        if fcurve is None:
                            fcurve = channelbag.fcurves.new(data_path, index=index)

                    group = ensure_group(group_name)
                    if group is not None and hasattr(fcurve, "group"):
                        try:
                            fcurve.group = group
                        except Exception:
                            pass
                    return fcurve

                # Location
                loc_fcurves = [
                    create_fcurve(
                        f'pose.bones["{bpy.utils.escape_identifier(bone_name)}"].location',
                        i,
                        bone_name,
                    )
                    for i in range(3)
                ]
                # Rotation
                rot_fcurves = [
                    create_fcurve(
                        f'pose.bones["{bpy.utils.escape_identifier(bone_name)}"].rotation_quaternion',
                        i,
                        bone_name,
                    )
                    for i in range(4)
                ]
                # Scale
                scale_fcurves = [
                    create_fcurve(
                        f'pose.bones["{bpy.utils.escape_identifier(bone_name)}"].scale',
                        i,
                        bone_name,
                    )
                    for i in range(3)
                ]

                # Most Roblox easings map to their corresponding interpolation type in Blender.
                interpolation_map = {
                    "Linear": "LINEAR",
                    "Constant": "CONSTANT",
                    "Elastic": "ELASTIC",
                    "Bounce": "BOUNCE",
                    "Sine": "SINE",
                    "Quad": "QUAD",
                    "Cubic": "CUBIC",
                    "CubicV2": "CUBIC",
                    "Quart": "QUART",
                    "Quint": "QUINT",
                    "Expo": "EXPO",
                    "Circular": "CIRC",
                    "Back": "BACK",
                }

                num_frames = len(sorted_frames)
                if num_frames == 0:
                    continue

                for fcurve in loc_fcurves + rot_fcurves + scale_fcurves:
                    fcurve.keyframe_points.add(num_frames)

                for idx, frame in enumerate(sorted_frames):
                    transforms = frame_data[frame]
                    loc = transforms["location"]
                    rot = transforms["rotation_quaternion"]
                    scl = transforms["scale"]

                    for axis in range(3):
                        kp = loc_fcurves[axis].keyframe_points[idx]
                        kp.co = (frame, loc[axis])
                        kp.handle_left_type = kp.handle_right_type = "AUTO"
                    for axis in range(4):
                        kp = rot_fcurves[axis].keyframe_points[idx]
                        kp.co = (frame, rot[axis])
                        kp.handle_left_type = kp.handle_right_type = "AUTO"
                    for axis in range(3):
                        kp = scale_fcurves[axis].keyframe_points[idx]
                        kp.co = (frame, scl[axis])
                        kp.handle_left_type = kp.handle_right_type = "AUTO"

                if num_frames > 1:
                    for idx in range(num_frames - 1):
                        current_frame_time = sorted_frames[idx]
                        current_transforms = frame_data[current_frame_time]
                        easing_style = current_transforms.get("easingStyle", "Linear")
                        easing_direction = current_transforms.get("easingDirection", "In")

                        for fcurve in loc_fcurves + rot_fcurves + scale_fcurves:
                            kp_current = fcurve.keyframe_points[idx]
                            kp_current.interpolation = interpolation_map.get(
                                easing_style, "LINEAR"
                            )

                            if kp_current.interpolation not in ["LINEAR", "CONSTANT"]:
                                if easing_direction == "In":
                                    kp_current.easing = "EASE_IN"
                                elif easing_direction == "Out":
                                    kp_current.easing = "EASE_OUT"
                                elif easing_direction == "InOut":
                                    kp_current.easing = "EASE_IN_OUT"
                            else:
                                kp_current.easing = "AUTO"

                for fcurve in loc_fcurves + rot_fcurves + scale_fcurves:
                    fcurve.update()

            pending_responses[task_id] = (True, "Animation imported successfully")

        if _is_mrxen0_r15_rig(ao):
            # MrXen0's rig has its own PROPERTIES-driven FK/IK system.
            # The generic IK preservation pass would clear FK curves on IK chains,
            # which is exactly where this retarget writes the imported motion.
            _do_import()
        else:
            # Run import wrapped with IK preservation (FK import then bake back to IK)
            import_animation_preserve_ik(_do_import)

    except Exception as e:
        print(f"Error in main thread (import_animation): {str(e)}")
        traceback.print_exc()
        pending_responses[task_id] = (False, f"Error importing animation: {str(e)}")


def execute_get_bone_rest(task_id, armature_name):
    """
    Gets the rest pose for all bones in an armature.
    This function uses the same rest pose calculation as the deform bone
    serializer to ensure perfect consistency between the rig setup
    and animation export.
    """
    original_mode = None
    ao = None
    saved_bone_matrices = {}

    try:
        if not armature_name:
            pending_responses[task_id] = (False, "No armature selected")
            return

        ao = get_object_by_name(armature_name)
        if not ao:
            pending_responses[task_id] = (
                False,
                f"Object '{armature_name}' is not a valid armature.",
            )
            return

        if ao.type != "ARMATURE":
            pending_responses[task_id] = (
                False,
                f"Object '{armature_name}' is not a valid armature.",
            )
            return

        from ..core.constants import get_transform_to_blender

        back_trans = get_transform_to_blender().inverted()
        world_transform = back_trans @ ao.matrix_world

        bone_poses = {}

        original_mode = ao.mode
        bpy.context.view_layer.objects.active = ao
        if ao.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")

        # Save current bone poses before clearing them
        for bone in ao.pose.bones:
            saved_bone_matrices[bone.name] = bone.matrix_basis.copy()

        # Clear transforms to guarantee we are calculating from the rest pose.
        # This is crucial for ensuring that pose_bone.matrix reflects the actual
        # rest pose of the armature.
        bpy.ops.pose.select_all(action="SELECT")
        bpy.ops.pose.transforms_clear()
        bpy.ops.pose.select_all(action="DESELECT")

        # Cache for rest transforms to avoid re-calculating for parents
        rest_transform_cache = {}

        # Iterate through bones sorted by hierarchy to ensure parents are processed before their children.
        # This is essential for correctly calculating parent-relative transforms.
        sorted_bones = sorted(ao.pose.bones, key=lambda b: len(b.parent_recursive))

        for pose_bone in sorted_bones:
            has_motor6d_props = (
                "transform" in pose_bone.bone
                and "transform1" in pose_bone.bone
                and "nicetransform" in pose_bone.bone
            )

            if has_motor6d_props:
                # Keep sync-bones aligned with the Motor6D serializer by rebuilding
                # the rest joint frame from the stored Roblox-space transforms.
                orig_mat = Matrix(pose_bone.bone["transform"])
                orig_mat_tr1 = Matrix(pose_bone.bone["transform1"])
                rest_obj_transform = back_trans @ (orig_mat @ orig_mat_tr1)
            else:
                # New/deform bones don't have stored Motor6D metadata, so derive the
                # rest frame from the actual Blender rest pose in object space.
                rest_obj_transform = world_transform @ pose_bone.bone.matrix_local

            rest_transform_cache[pose_bone.name] = rest_obj_transform

            # Calculate the bone's rest transform relative to its parent.
            reported_parent_name = _get_reported_bone_parent_name(pose_bone)
            if reported_parent_name:
                parent_rest_transform = rest_transform_cache.get(reported_parent_name)
                if parent_rest_transform:
                    try:
                        # The relative transform is the transformation from the parent's space to the child's space.
                        rest_local_transform = (
                            parent_rest_transform.inverted() @ rest_obj_transform
                        )
                    except ValueError:
                        # Fallback for non-invertible parent matrix, though this is rare.
                        rest_local_transform = rest_obj_transform
                else:
                    # This case should not be hit with presorted bones, but serves as a safe fallback.
                    rest_local_transform = rest_obj_transform
            else:
                # For root bones, the local transform is the same as its object-space transform.
                rest_local_transform = rest_obj_transform

            world_matrix = rest_obj_transform
            relative_matrix = rest_local_transform

            # Convert matrices to Roblox CFrame-compatible components.
            world_translation = world_matrix.to_translation()
            world_components = [
                world_translation.x,
                world_translation.y,
                world_translation.z,
                world_matrix[0][0],
                world_matrix[0][1],
                world_matrix[0][2],
                world_matrix[1][0],
                world_matrix[1][1],
                world_matrix[1][2],
                world_matrix[2][0],
                world_matrix[2][1],
                world_matrix[2][2],
            ]

            relative_translation = relative_matrix.to_translation()
            relative_components = [
                relative_translation.x,
                relative_translation.y,
                relative_translation.z,
                relative_matrix[0][0],
                relative_matrix[0][1],
                relative_matrix[0][2],
                relative_matrix[1][0],
                relative_matrix[1][1],
                relative_matrix[1][2],
                relative_matrix[2][0],
                relative_matrix[2][1],
                relative_matrix[2][2],
            ]

            is_synthetic = pose_bone.bone.get("is_synthetic_helper", False)
            bone_poses[pose_bone.name] = {
                "world": world_components,
                "relative": relative_components,
                "parent": reported_parent_name,
                "is_synthetic_helper": is_synthetic,
            }

        # Restore original mode
        if ao.mode != original_mode:
            bpy.ops.object.mode_set(mode=original_mode)

        response = {"armature": armature_name, "bone_poses": bone_poses}

        data = json.dumps(response).encode("utf-8")
        pending_responses[task_id] = (True, data)

    except Exception as e:
        traceback.print_exc()
        pending_responses[task_id] = (False, f"Error getting bone rest poses: {str(e)}")
    finally:
        # Restore original bone poses
        if ao and saved_bone_matrices:
            for bone in ao.pose.bones:
                if bone.name in saved_bone_matrices:
                    bone.matrix_basis = saved_bone_matrices[bone.name]

        # Ensure mode is restored even if an error occurs
        if original_mode and ao and ao.mode != original_mode:
            bpy.ops.object.mode_set(mode=original_mode)
