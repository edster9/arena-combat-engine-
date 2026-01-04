#!/usr/bin/env python3
"""
Blender Python script to extract vehicle parts from a combined OBJ file.

Extracts:
  - Body parts (body, spoiler, etc.) -> chassis OBJ file
  - Single wheel (centered at origin) -> wheel OBJ file

Usage:
  blender --background --python blender_extract_vehicle_parts.py -- \
      input.obj output_body.obj output_wheel.obj

Or run from within Blender's scripting interface.
"""

import bpy
import sys
import os
from mathutils import Vector


def clear_scene():
    """Remove all objects from the scene."""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)


def import_obj(filepath):
    """Import an OBJ file, splitting by groups."""
    bpy.ops.wm.obj_import(filepath=filepath, import_vertex_groups=True)


def separate_by_vertex_groups(obj):
    """Separate a mesh into multiple objects based on vertex groups."""
    if not obj.vertex_groups:
        print(f"  No vertex groups found in {obj.name}")
        return []

    print(f"  Found {len(obj.vertex_groups)} vertex groups in {obj.name}:")
    for vg in obj.vertex_groups:
        print(f"    - {vg.name}")

    new_objects = []

    # For each vertex group, create a separate object
    for vg in list(obj.vertex_groups):
        # Deselect all
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        # Enter edit mode
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')

        # Select vertices in this group
        bpy.ops.object.mode_set(mode='OBJECT')
        obj.vertex_groups.active = vg

        # Select vertices by weight
        for v in obj.data.vertices:
            for g in v.groups:
                if g.group == vg.index and g.weight > 0.5:
                    v.select = True
                    break

        bpy.ops.object.mode_set(mode='EDIT')

        # Check if any vertices selected
        bpy.ops.object.mode_set(mode='OBJECT')
        selected_count = sum(1 for v in obj.data.vertices if v.select)

        if selected_count > 0:
            bpy.ops.object.mode_set(mode='EDIT')
            # Separate selected vertices into new object
            bpy.ops.mesh.separate(type='SELECTED')
            bpy.ops.object.mode_set(mode='OBJECT')

            # Find the newly created object (it will be selected)
            for new_obj in bpy.context.selected_objects:
                if new_obj != obj:
                    new_obj.name = vg.name
                    new_objects.append(new_obj)
                    print(f"  Created object: {vg.name}")
        else:
            bpy.ops.object.mode_set(mode='OBJECT')

    return new_objects


def get_objects_by_name_pattern(patterns):
    """Get objects whose names contain any of the given patterns."""
    result = []
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            name_lower = obj.name.lower()
            for pattern in patterns:
                if pattern.lower() in name_lower:
                    result.append(obj)
                    break
    return result


def select_objects(objects):
    """Select only the specified objects."""
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    if objects:
        bpy.context.view_layer.objects.active = objects[0]


def export_selected_obj(filepath):
    """Export selected objects to OBJ file with UVs for texturing."""
    bpy.ops.wm.obj_export(
        filepath=filepath,
        export_selected_objects=True,
        export_materials=True,
        export_uv=True,
        export_normals=True,
        export_triangulated_mesh=True,
        forward_axis='NEGATIVE_Z',
        up_axis='Y'
    )


def center_object_at_origin(obj):
    """Move object so its center is at the world origin."""
    # Get bounding box center in world space
    bbox_corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    center = sum(bbox_corners, Vector()) / 8

    # Move object
    obj.location -= center

    # Apply the transform to mesh data
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)


def duplicate_object(obj, new_name):
    """Duplicate an object with a new name."""
    # Duplicate
    new_obj = obj.copy()
    new_obj.data = obj.data.copy()
    new_obj.name = new_name
    bpy.context.collection.objects.link(new_obj)
    return new_obj


def extract_vehicle_parts(input_path, body_output_path, wheel_output_path):
    """
    Main extraction function.

    Args:
        input_path: Path to source OBJ file
        body_output_path: Output path for body mesh
        wheel_output_path: Output path for wheel mesh
    """
    print(f"\n=== Vehicle Part Extractor ===")
    print(f"Input:  {input_path}")
    print(f"Body:   {body_output_path}")
    print(f"Wheel:  {wheel_output_path}")

    # Clear and import
    clear_scene()
    import_obj(input_path)

    # List all imported objects
    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    print(f"\nImported {len(mesh_objects)} mesh object(s):")
    for obj in mesh_objects:
        print(f"  - {obj.name}")
        if obj.vertex_groups:
            print(f"    (has {len(obj.vertex_groups)} vertex groups)")

    # If only one object with vertex groups, separate by groups
    if len(mesh_objects) == 1 and mesh_objects[0].vertex_groups:
        print(f"\nSeparating by vertex groups...")
        separate_by_vertex_groups(mesh_objects[0])

        # Refresh mesh objects list
        mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
        print(f"\nAfter separation: {len(mesh_objects)} objects")
        for obj in mesh_objects:
            print(f"  - {obj.name}")

    # Find body parts (body, spoiler, chassis, etc. - NOT wheels)
    body_patterns = ['body', 'spoiler', 'chassis', 'hood', 'roof', 'door', 'bumper', 'fender', 'trunk']
    wheel_patterns = ['wheel']

    body_objects = get_objects_by_name_pattern(body_patterns)
    wheel_objects = get_objects_by_name_pattern(wheel_patterns)

    # If no body objects found by pattern, take everything that's not a wheel
    if not body_objects:
        body_objects = [obj for obj in bpy.context.scene.objects
                        if obj.type == 'MESH' and obj not in wheel_objects]

    print(f"\nBody objects ({len(body_objects)}):")
    for obj in body_objects:
        print(f"  - {obj.name}")

    print(f"\nWheel objects ({len(wheel_objects)}):")
    for obj in wheel_objects:
        print(f"  - {obj.name}")

    # Export body parts
    if body_objects:
        select_objects(body_objects)
        export_selected_obj(body_output_path)
        print(f"\n✓ Exported body to: {body_output_path}")
    else:
        print(f"\n✗ No body objects found!")

    # Export a single wheel, centered at origin
    if wheel_objects:
        # Use the first wheel (e.g., front-left)
        source_wheel = wheel_objects[0]
        print(f"\nUsing wheel: {source_wheel.name}")

        # Duplicate and center it
        centered_wheel = duplicate_object(source_wheel, "wheel_centered")
        center_object_at_origin(centered_wheel)

        # Select only the centered wheel
        select_objects([centered_wheel])
        export_selected_obj(wheel_output_path)
        print(f"✓ Exported wheel to: {wheel_output_path}")

        # Clean up
        bpy.data.objects.remove(centered_wheel, do_unlink=True)
    else:
        print(f"\n✗ No wheel objects found!")

    print("\n=== Done ===\n")


def main():
    """Parse command line args and run extraction."""
    # Get args after "--"
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        # Default paths for testing
        script_dir = os.path.dirname(os.path.realpath(__file__))
        project_root = os.path.dirname(script_dir)
        argv = [
            os.path.join(project_root, "assets/models/vehicles/sedan.obj"),
            os.path.join(project_root, "assets/models/chassis/compact_body.obj"),
            os.path.join(project_root, "assets/models/wheels/wheel_standard.obj"),
        ]

    if len(argv) < 3:
        print("Usage: blender --background --python blender_extract_vehicle_parts.py -- "
              "input.obj body_output.obj wheel_output.obj")
        sys.exit(1)

    input_path = os.path.abspath(argv[0])
    body_output = os.path.abspath(argv[1])
    wheel_output = os.path.abspath(argv[2])

    # Create output directories if needed
    os.makedirs(os.path.dirname(body_output), exist_ok=True)
    os.makedirs(os.path.dirname(wheel_output), exist_ok=True)

    extract_vehicle_parts(input_path, body_output, wheel_output)


if __name__ == "__main__":
    main()
