"""Export the demonstration rover's CAD meshes with independent wheel/gimbal pivots.

Run with Blender: blender -b --python tools/export_rover_model.py -- VIDEO_SOURCE OUTPUT_JS
VIDEO_SOURCE is the original video's 工程源文件 directory.
"""
import argparse
from array import array
from base64 import b64encode
from collections import defaultdict
import json
from pathlib import Path
import sys

import bpy


def web_axes(value):
    # Blender +X forward, +Y left, +Z up; Three.js +Z forward, +X left, +Y up.
    return (value.y, value.z, value.x)


def encoded(kind, values):
    numbers = array(kind, values)
    if sys.byteorder != 'little':
        numbers.byteswap()
    return b64encode(numbers).decode('ascii')


def collect_geometry(nodes, rover):
    deps = bpy.context.evaluated_depsgraph_get()
    groups = defaultdict(dict)
    material_ids = {}
    for obj in rover['parts']:
        if obj.type not in ('MESH', 'CURVE') or obj.parent is None:
            continue
        pivot = obj.parent
        while pivot not in nodes and pivot.parent is not None:
            pivot = pivot.parent
        if pivot not in nodes or nodes[pivot] is None:
            continue
        evaluated = obj.evaluated_get(deps)
        mesh = evaluated.to_mesh()
        if not mesh:
            continue
        transform = pivot.matrix_world.inverted() @ obj.matrix_world
        normal_transform = transform.to_3x3().inverted().transposed()
        try:
            mesh.calc_loop_triangles()
            coordinates = [tuple(round(axis * 40000) for axis in web_axes(transform @ v.co))
                           for v in mesh.vertices]
            normals = [tuple(round(axis * 127) for axis in web_axes((normal_transform @ n.vector).normalized()))
                       for n in mesh.corner_normals]
            for triangle in mesh.loop_triangles:
                material = mesh.materials[triangle.material_index]
                mat_id = material_ids.setdefault(material, len(material_ids))
                batch = groups[nodes[pivot]].setdefault(mat_id, {'lookup': {}, 'p': [], 'n': [], 'i': []})
                keys = [coordinates[v] + normals[loop] for v, loop in zip(triangle.vertices, triangle.loops)]
                if len({key[:3] for key in keys}) < 3:
                    continue
                for key in keys:
                    if key not in batch['lookup']:
                        batch['lookup'][key] = len(batch['lookup'])
                        batch['p'].extend(key[:3])
                        batch['n'].extend(key[3:])
                    batch['i'].append(batch['lookup'][key])
        finally:
            evaluated.to_mesh_clear()
    materials = []
    for material in material_ids:
        inputs = material.node_tree.nodes.get('Principled BSDF').inputs
        materials.append([*(round(n, 4) for n in inputs['Base Color'].default_value[:3]),
                          round(inputs['Roughness'].default_value, 3),
                          round(inputs['Metallic'].default_value, 3)])
    result = {}
    for name, batches in groups.items():
        result[name] = [
            [mat_id, encoded('h', batch['p']), encoded('b', batch['n']), encoded('H', batch['i'])]
            for mat_id, batch in batches.items() if batch['i']
        ]
        print(name, sum(len(batch['i']) // 3 for batch in batches.values()), 'triangles', flush=True)
    return materials, result


def simplify_cad(rover):
    for obj in rover['meshes']:
        if not obj.get('cad_source'):
            continue
        if obj in rover['shell_meshes']:
            ratio = .31
        elif 'Wheel_' in obj.name:
            ratio = .14
        else:
            ratio = .22
        modifier = obj.modifiers.new('Web mesh simplification', 'DECIMATE')
        modifier.ratio = ratio


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video_source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    cad = args.video_source / 'cad-assets'
    required = ('rover_shell_real.obj', 'left_97mm_mecanum.obj', 'right_97mm_mecanum.obj',
                'gimbal_carrier_disc.obj')
    for name in required:
        if not (cad / name).is_file():
            raise FileNotFoundError(cad / name)
    sys.path.insert(0, str(args.video_source / 'scene'))
    from rover_model import build_rover
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rover = build_rover(cad / required[0])
    simplify_cad(rover)
    bpy.context.view_layer.update()
    fl, fr, rl, rr = rover['wheels']
    nodes = {rover['body']: 'body', rover['pan']: 'pan', rover['pitch']: 'pitch',
             fl: 'leftHelix', fr: 'rightHelix', rl: None, rr: None}
    materials, geometry = collect_geometry(nodes, rover)
    payload = {
        'materials': materials,
        'geometry': geometry,
        'pan': web_axes(rover['pan'].location),
        'pitch': web_axes(rover['pitch'].location),
        # Center between the two transducer faces from the video model.
        'sensor': (0, .028, .036),
        'wheels': [web_axes(wheel.location) for wheel in rover['wheels']],
    }
    with args.output.open('w', encoding='ascii', newline='\n') as stream:
        stream.write('// Generated by tools/export_rover_model.py from the demonstration rover.\n')
        stream.write('globalThis.RoverModelData = ')
        json.dump(payload, stream, separators=(',', ':'))
        stream.write(';\n')
    print(args.output, args.output.stat().st_size, 'bytes', flush=True)


if __name__ == '__main__':
    main()
