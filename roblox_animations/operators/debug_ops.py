"""Debug operators for inspecting rig import/binding state."""

import json
import re

import bpy
from mathutils import Matrix, Vector

from ..core.constants import get_transform_to_blender
from ..core.utils import (
    cf_to_mat,
    find_master_collection_for_object,
    find_parts_collection_in_master,
    get_object_by_name,
)


def _fmt_vec(vec):
    return f"({vec.x:.4f},{vec.y:.4f},{vec.z:.4f})"


def _strip_suffix(name):
    return re.sub(r"\.\d+$", "", name or "")


def _iter_rig_nodes(node, parent_name=None):
    if not isinstance(node, dict):
        return
    yield node, parent_name
    name = node.get("jname") or parent_name
    for child in node.get("children") or []:
        yield from _iter_rig_nodes(child, name)


def _expected_node_head(node):
    transform = node.get("transform")
    if not transform:
        return None
    try:
        matrix = cf_to_mat(transform)
        joint_transform = node.get("jointtransform1") or node.get("jointTransform1")
        if joint_transform:
            matrix = matrix @ cf_to_mat(joint_transform)
        return (get_transform_to_blender() @ matrix).to_translation()
    except Exception:
        return None


def _matrix_from_idprop(value):
    try:
        return Matrix(value)
    except Exception:
        return None


def _matrix_deviation_from_identity(value):
    matrix = _matrix_from_idprop(value)
    if matrix is None:
        return None

    translation_error = matrix.to_translation().length
    rotation_error = 0.0
    rotation = matrix.to_3x3()
    identity = Matrix.Identity(3)
    for row in range(3):
        for col in range(3):
                rotation_error += abs(rotation[row][col] - identity[row][col])
    return translation_error, rotation_error


def _resolve_armature(context):
    settings = getattr(context.scene, "rbx_anim_settings", None)
    armature_name = getattr(settings, "rbx_anim_armature", "") if settings else ""
    armature = get_object_by_name(armature_name, context.scene) if armature_name else None
    if armature and armature.type == "ARMATURE":
        return armature

    active = context.view_layer.objects.active if context.view_layer else None
    if active and active.type == "ARMATURE":
        return active

    for obj in context.scene.objects:
        if obj.type == "ARMATURE":
            return obj
    return None


def _find_meta_object(master_collection):
    if not master_collection:
        return None
    for obj in master_collection.objects:
        if "RigMeta" in obj:
            return obj
    return None


def _mesh_weight_summary(obj, armature):
    bone_names = {bone.name for bone in armature.data.bones}
    group_names = {
        group.index: group.name
        for group in obj.vertex_groups
        if group.name in bone_names
    }
    totals = {}
    assigned_vertices = 0

    if obj.type != "MESH" or obj.data is None:
        return assigned_vertices, 0, []

    for vertex in obj.data.vertices:
        vertex_assigned = False
        for group_ref in vertex.groups:
            group_name = group_names.get(group_ref.group)
            if not group_name or group_ref.weight <= 0:
                continue
            totals[group_name] = totals.get(group_name, 0.0) + float(group_ref.weight)
            vertex_assigned = True
        if vertex_assigned:
            assigned_vertices += 1

    top_groups = sorted(totals.items(), key=lambda item: item[1], reverse=True)[:6]
    return assigned_vertices, len(obj.data.vertices), top_groups


def _bone_chain(armature, bone_name):
    bone = armature.data.bones.get(bone_name)
    if bone is None:
        return None

    names = []
    while bone is not None:
        names.append(bone.name)
        bone = bone.parent
    names.reverse()
    return ">".join(names)


def _matrix_delta(value):
    if value is None:
        return None

    try:
        translation_error = value.to_translation().length
        rotation_error = 0.0
        rotation = value.to_3x3()
        identity = Matrix.Identity(3)
        for row in range(3):
            for col in range(3):
                rotation_error += abs(rotation[row][col] - identity[row][col])
        return translation_error, rotation_error
    except Exception:
        return None


def _vertex_weight_for_group(vertex, group_index):
    for group_ref in vertex.groups:
        if group_ref.group == group_index:
            return float(group_ref.weight)
    return 0.0


def _weighted_group_centroid(obj, group_name, depsgraph=None, evaluated=False):
    if obj.type != "MESH" or obj.data is None:
        return None

    group = obj.vertex_groups.get(group_name)
    if group is None:
        return None

    source_vertices = obj.data.vertices
    weights = [
        (vertex.index, _vertex_weight_for_group(vertex, group.index))
        for vertex in source_vertices
    ]
    weights = [(index, weight) for index, weight in weights if weight > 0.0]
    if not weights:
        return None

    eval_mesh = None
    eval_obj = obj
    try:
        if evaluated and depsgraph is not None:
            eval_obj = obj.evaluated_get(depsgraph)
            eval_mesh = eval_obj.to_mesh()
            vertices = eval_mesh.vertices
        else:
            vertices = obj.data.vertices

        total_weight = 0.0
        centroid = Vector((0.0, 0.0, 0.0))
        for index, weight in weights:
            if index >= len(vertices):
                continue
            centroid += (eval_obj.matrix_world @ vertices[index].co) * weight
            total_weight += weight

        if total_weight <= 0.0:
            return None
        return centroid / total_weight, len(weights), total_weight
    finally:
        if eval_mesh is not None:
            try:
                eval_obj.to_mesh_clear()
            except Exception:
                pass


def _append_pose_probe(lines, context, armature, mesh_objects):
    depsgraph = context.evaluated_depsgraph_get()
    focus_terms = ("arm", "leg", "torso", "root", "hand", "shoulder", "sword", "claw")
    focus_bones = []

    for pose_bone in armature.pose.bones:
        name_lower = pose_bone.name.lower()
        is_focus = any(term in name_lower for term in focus_terms)
        basis_delta = _matrix_delta(pose_bone.matrix_basis)
        basis_loc = basis_delta[0] if basis_delta else 0.0
        basis_rot = basis_delta[1] if basis_delta else 0.0
        if is_focus or basis_loc > 0.0001 or basis_rot > 0.0001:
            focus_bones.append((pose_bone, basis_loc, basis_rot))

    if not focus_bones:
        return

    for pose_bone, basis_loc, basis_rot in focus_bones:
        rest_bone = armature.data.bones.get(pose_bone.name)
        if rest_bone is None:
            continue
        rest_head = armature.matrix_world @ rest_bone.head_local
        rest_tail = armature.matrix_world @ rest_bone.tail_local
        pose_head = armature.matrix_world @ pose_bone.head
        pose_tail = armature.matrix_world @ pose_bone.tail
        head_move = (pose_head - rest_head).length
        tail_move = (pose_tail - rest_tail).length
        if basis_loc <= 0.0001 and basis_rot <= 0.0001 and tail_move <= 0.0001:
            continue
        lines.append(
            f"[RBXDIAG] POSE bone={pose_bone.name} basis_loc={basis_loc:.5f} "
            f"basis_rot={basis_rot:.5f} head_move={head_move:.5f} tail_move={tail_move:.5f} "
            f"pose_head={_fmt_vec(pose_head)} pose_tail={_fmt_vec(pose_tail)}"
        )

        affected = []
        for obj in mesh_objects:
            rest_centroid = _weighted_group_centroid(obj, pose_bone.name, depsgraph, evaluated=False)
            if rest_centroid is None:
                continue
            eval_centroid = _weighted_group_centroid(obj, pose_bone.name, depsgraph, evaluated=True)
            if eval_centroid is None:
                continue
            rest_pos, count, total_weight = rest_centroid
            eval_pos, _eval_count, _eval_weight = eval_centroid
            moved = (eval_pos - rest_pos).length
            distance_to_head = (eval_pos - pose_head).length
            affected.append((moved, obj.name, count, total_weight, rest_pos, eval_pos, distance_to_head))

        for moved, obj_name, count, total_weight, rest_pos, eval_pos, distance_to_head in sorted(
            affected,
            key=lambda item: item[0],
            reverse=True,
        )[:8]:
            lines.append(
                f"[RBXDIAG] DEFORM bone={pose_bone.name} mesh={obj_name} verts={count} "
                f"weight={total_weight:.2f} moved={moved:.5f} dist_to_head={distance_to_head:.5f} "
                f"rest_centroid={_fmt_vec(rest_pos)} eval_centroid={_fmt_vec(eval_pos)}"
            )


def build_rig_binding_diagnostics(context):
    lines = ["[RBXDIAG] === Rig binding diagnostics ==="]
    armature = _resolve_armature(context)
    if not armature:
        lines.append("[RBXDIAG] ERROR: no armature found")
        return "\n".join(lines)

    master_collection = find_master_collection_for_object(armature)
    parts_collection = find_parts_collection_in_master(master_collection) if master_collection else None
    meta_obj = _find_meta_object(master_collection)

    lines.append(f"[RBXDIAG] Armature: {armature.name}, bones={len(armature.data.bones)}")
    lines.append(f"[RBXDIAG] Master collection: {master_collection.name if master_collection else '<none>'}")
    lines.append(f"[RBXDIAG] Parts collection: {parts_collection.name if parts_collection else '<none>'}")

    if meta_obj:
        try:
            meta = json.loads(meta_obj["RigMeta"])
        except Exception as exc:
            lines.append(f"[RBXDIAG] Meta: {meta_obj.name}, parse_error={exc}")
            meta = None
        else:
            lines.append(
                f"[RBXDIAG] Meta: {meta_obj.name}, rigName={meta.get('rigName')}, "
                f"partAux={len(meta.get('partAux') or [])}"
            )

        if meta:
            focus_terms = ("arm", "leg", "torso", "root", "hand", "shoulder", "sword", "claw")
            for node, parent_name in _iter_rig_nodes(meta.get("rig")):
                name = node.get("jname")
                if not name:
                    continue
                bone = armature.data.bones.get(name)
                expected_head = _expected_node_head(node)
                if not bone or expected_head is None:
                    lines.append(f"[RBXDIAG] BONE-MISSING name={name} parent={parent_name}")
                    continue
                actual_head = bone.head_local
                delta = (actual_head - expected_head).length
                skin_delta = None
                skin_matrix = _matrix_from_idprop(bone.get("rbx_skin_bind_rest"))
                if skin_matrix is not None:
                    skin_delta = (actual_head - skin_matrix.to_translation()).length
                lower_name = name.lower()
                is_focus = any(term in lower_name for term in focus_terms)
                if is_focus or delta > 0.01 or skin_delta is not None:
                    chain = _bone_chain(armature, name) or "-"
                    nice_delta = _matrix_deviation_from_identity(bone.get("nicetransform"))
                    nice_text = ""
                    if nice_delta is not None:
                        nice_text = f" nice_loc={nice_delta[0]:.5f} nice_rot={nice_delta[1]:.5f}"
                    skin_text = ""
                    if skin_delta is not None:
                        skin_text = f" skin_delta={skin_delta:.5f}"
                    lines.append(
                        f"[RBXDIAG] BONE {name} parent={bone.parent.name if bone.parent else '<none>'} "
                        f"meta_parent={parent_name or '<none>'} delta={delta:.5f} "
                        f"expected={_fmt_vec(expected_head)} actual={_fmt_vec(actual_head)} "
                        f"jointType={node.get('jointType')} deform={bool(node.get('isDeformBone'))} "
                        f"chain={chain}{nice_text}{skin_text}"
                    )
    else:
        lines.append("[RBXDIAG] Meta: <none>")

    mesh_objects = []
    if parts_collection:
        mesh_objects = [obj for obj in parts_collection.objects if obj.type == "MESH"]
    elif master_collection:
        mesh_objects = [obj for obj in master_collection.objects if obj.type == "MESH"]
    else:
        mesh_objects = [obj for obj in context.scene.objects if obj.type == "MESH"]

    skinned = rigid = both = unbound = 0
    for obj in sorted(mesh_objects, key=lambda item: item.name):
        arm_mods = [
            mod.name
            for mod in obj.modifiers
            if mod.type == "ARMATURE" and getattr(mod, "object", None) == armature
        ]
        child_of = [
            c.subtarget
            for c in obj.constraints
            if c.type == "CHILD_OF" and c.target == armature
        ]
        assigned, total, top_groups = _mesh_weight_summary(obj, armature)
        has_skin = bool(arm_mods and assigned > 0)
        has_rigid = bool(child_of)

        if has_skin and has_rigid:
            state = "BOTH"
            both += 1
        elif has_skin:
            state = "SKINNED"
            skinned += 1
        elif has_rigid:
            state = "RIGID"
            rigid += 1
        else:
            state = "UNBOUND"
            unbound += 1

        group_text = ", ".join(f"{name}:{weight:.2f}" for name, weight in top_groups) or "-"
        chain_text = "; ".join(
            f"{name}:{_bone_chain(armature, name) or '-'}"
            for name, _weight in top_groups
        ) or "-"
        child_text = ",".join(child_of) or "-"
        mod_text = ",".join(arm_mods) or "-"
        warn = ""
        if state == "RIGID" and any("arm" in target.lower() or "leg" in target.lower() for target in child_of):
            warn = " WARN rigid-limb"
        if state == "BOTH":
            warn = " WARN skin-and-rigid"

        lines.append(
            f"[RBXDIAG] MESH {obj.name} base={_strip_suffix(obj.name)} state={state}{warn} "
            f"verts={assigned}/{total} armature_mod={mod_text} child_of={child_text} "
            f"top_groups={group_text} group_chains={chain_text}"
        )

    lines.append(
        f"[RBXDIAG] Summary meshes={len(mesh_objects)} "
        f"skinned={skinned} rigid={rigid} both={both} unbound={unbound}"
    )
    _append_pose_probe(lines, context, armature, mesh_objects)
    return "\n".join(lines)


class OBJECT_OT_DebugRigBindings(bpy.types.Operator):
    bl_idname = "object.rbxanims_debug_rig_bindings"
    bl_label = "Debug Rig Bindings"
    bl_description = "Print mesh, armature, constraint, and vertex-weight diagnostics for the selected Roblox rig"

    def execute(self, context):
        report = build_rig_binding_diagnostics(context)
        print(report)
        try:
            context.window_manager.clipboard = report
        except Exception:
            pass
        self.report({"INFO"}, "Rig binding diagnostics printed and copied to clipboard")
        return {"FINISHED"}
