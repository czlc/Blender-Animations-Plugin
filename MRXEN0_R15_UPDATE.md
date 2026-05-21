# MrXen0 Roblox R15 FK/IK Rig Import Update

Date: 2026-05-21

This document summarizes the import changes made for the MrXen0 "Roblox Blocky R15 RIG" v1.2 Blender rig.

## Background

The existing Roblox animation import path worked for clean/generated R15 armatures, but the MrXen0 rig has a different structure:

- FK control bones such as `FK_UpperArm.L`, `FK_Foot.R`
- torso/head controls such as `LowTorso`, `UpTorso`, `HEAD`
- helper/MCH bones and constraints
- rig properties on `PROPERTIES`, including `ARM_IK_FK.*`, `LEG_IK_FK.*`, and `HEAD_FOLLOW`

Roblox `KeyFrameSequence` data stores R15 `Pose.CFrame` values in Roblox part/local pose space. Those values cannot be written directly to the MrXen0 FK control bones, because the FK control bones have different rest axes and are connected through constraints.

## Final Behavior

When the selected Blender armature is detected as the MrXen0 R15 rig, import now uses a dedicated FK retarget path.

Detection is isolated through `_is_mrxen0_r15_rig()`:

- preferred: armature data has `rig_id == "xqpb4vnl0zay"`
- fallback: the armature contains the expected MrXen0 FK bones and `PROPERTIES`

The current diagnostic revision marker is:

```text
fk_head_control_probe_20260521
```

## Key Changes

### Preserve Rig Drivers

For MrXen0 rigs, import no longer calls `animation_data_clear()`.

That call removed rig drivers and caused FK controls to move while the mesh did not. The importer now clears only the active action:

```text
ao.animation_data.action = None
```

Generic/non-MrXen0 rigs still use the original clearing behavior.

### FK Control Mapping

Roblox R15 source bones are mapped to MrXen0 controls:

```text
LowerTorso -> LowTorso
UpperTorso -> UpTorso
Head       -> HEAD

LeftUpperArm  -> FK_UpperArm.L
LeftLowerArm  -> FK_LowerArm.L
LeftHand      -> FK_Hand.L
RightUpperArm -> FK_UpperArm.R
RightLowerArm -> FK_LowerArm.R
RightHand     -> FK_Hand.R

LeftUpperLeg  -> FK_UpperLeg.L
LeftLowerLeg  -> FK_LowerLeg.L
LeftFoot      -> FK_Foot.L
RightUpperLeg -> FK_UpperLeg.R
RightLowerLeg -> FK_LowerLeg.R
RightFoot     -> FK_Foot.R
```

The head mapping was important: writing to `Head` produced a stable 12-15 degree error, while writing to `HEAD` brought head rotation error to zero.

### Force Import Rig Properties

During import, the MrXen0 rig is placed in FK mode:

```text
ARM_IK_FK.L = 0
ARM_IK_FK.R = 0
LEG_IK_FK.L = 0
LEG_IK_FK.R = 0
HEAD_FOLLOW = 1
```

These properties are keyed at the start frame.

### Hierarchical Source Pose Reconstruction

The importer reconstructs expected Roblox R15 source matrices in hierarchy order:

```text
LowerTorso -> UpperTorso -> Head / arms
LowerTorso -> legs
```

This fixed the earlier upper/lower body separation, where each source bone had been calculated independently.

### Virtual Part Rest Space For Limbs

Roblox limb `Pose.CFrame` values are authored in body-part pose space, not in the MrXen0 bone-chain axis space.

For limb bones, import now creates a virtual source rest matrix:

- arms/hands use `UpperTorso` rest rotation plus each limb bone's rest position
- legs/feet use `LowerTorso` rest rotation plus each limb bone's rest position

The result is then mapped back to the actual MrXen0 FK control rest space.

This fixed the issue where hands appeared behind the back instead of in front of the chest.

### Driven Bone Correction

After writing a FK control, the importer checks the actual driven Roblox-named bone, such as `RightHand` or `LeftUpperArm`.

If the driven bone does not match the expected source matrix, the importer applies a small correction back to the FK control. This is iterated a few times to account for constraints.

## Validation

The MrXen0 path now prints matrix verification data after import.

It samples the first, middle, and last imported frames, then compares:

```text
expected matrix from Roblox Pose.CFrame
vs
actual matrix of the driven Blender R15 bone
```

Latest observed result:

```text
samples=3
comparisons=45
max_loc=0.004000
avg_loc=0.001475
max_rot_deg=0.0000
avg_rot_deg=0.0000
```

Interpretation:

- all 15 R15 bones were mapped
- rotation matched exactly in sampled frames
- remaining translation error is very small and likely comes from constraint/bone endpoint evaluation

## Isolation From Existing Import Paths

The new behavior is only active when `_is_mrxen0_r15_rig()` returns true.

Other rigs continue to use the existing import behavior:

- clean/generated R15 armatures use the original transformable-bone path
- deform rigs use the existing deform bone path
- generic IK rigs still use `import_animation_preserve_ik()`

The MrXen0 path skips the generic IK preservation wrapper because that wrapper clears FK curves on IK chains, which conflicts with how this rig expects FK controls to be animated.

## Known Hard-Coded MrXen0 Details

This is not a general Blender retargeting solution. It is a targeted adapter for the MrXen0 R15 rig.

Hard-coded rig-specific details include:

- `rig_id = "xqpb4vnl0zay"`
- FK bone names
- `LowTorso`, `UpTorso`, and `HEAD`
- `PROPERTIES` custom properties
- Roblox R15 parent hierarchy
- virtual part rest-space rules for limbs

If another third-party R15 rig uses different control names or constraints, it should get its own adapter instead of sharing these mappings.

## Build Artifacts

The updated Blender addon is packaged using the original fixed names:

```text
rbx_anims_v2.5.1.zip
rbx_anims_v2.5.1_legacy.zip
```

The latest package should log:

```text
Blender Addon: MrXen0 R15 rig detected; importing Roblox R15 motion to FK controls. rev=fk_head_control_probe_20260521
```

