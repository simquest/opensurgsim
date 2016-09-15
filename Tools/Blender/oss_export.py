# Copyright 2016 SimQuest Solutions

bl_info = {
    "name": "OpenSurgSim FEM PLY format",
    "author": "Ryan Kornheisl, Paul Novotny",
    "location": "File > Import-Export",
    "description": "Export FEM PLY mesh data for use with OpenSurgSim",
    "support": 'COMMUNITY',
    "category": "Import-Export"}

import os
import bpy
from bpy.props import (CollectionProperty,
                       StringProperty,
                       BoolProperty,
                       EnumProperty,
                       FloatProperty,
                       )
from bpy_extras.io_utils import (ExportHelper,
                                 axis_conversion,
                                 )

class ExportPLY(bpy.types.Operator, ExportHelper):
    """Export a single object as a PLY file for use as an FEM in OpenSurgSim"""
    bl_idname = "export_mesh.fem"
    bl_label = "Export FEM for OpenSurgSim"

    filename_ext = ".ply"

    filter_glob = StringProperty(default="*.ply", options={'HIDDEN'})

    use_graphics = BoolProperty(
            name="Export Graphics",
            description="Include data for graphics meshes",
            default=False,
            )
    fem_dimensions = EnumProperty(
            name="FEM",
            items=(('1', "FEM 1D", ""),
                   ('2', "FEM 2D", ""),
                   ('3', "FEM 3D", ""),
                  ),
            default='2',
            )

    @classmethod
    def poll(cls, context):
        return context.active_object != None

    def execute(self, context):
        keywords = self.as_keywords(ignore=("check_existing",
                                            "filter_glob",
                                            ))

        filepath = self.filepath
        filepath = bpy.path.ensure_ext(filepath, self.filename_ext)

        return export_fem_save(self, context, **keywords)

    def draw(self, context):
        layout = self.layout

        row = layout.row()
        row.prop(self, "use_graphics")        

        layout.prop(self, "fem_dimensions")


def export_fem_save(operator,
         context,
         fem_dimensions,
         filepath="",
         use_graphics=False,
         ):

    scene = context.scene
    object = context.active_object

    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')

    mesh = object.data.copy()

    if not mesh:
        raise Exception("Error, could not get mesh data from active object")

    if use_graphics:
        mesh.calc_normals()

    ret = save_mesh(filepath, mesh, object.get('boundary_condition'),
                    fem_dimensions,
                    use_graphics=use_graphics,
                    )

    bpy.data.meshes.remove(mesh)

    return ret


def save_mesh(filepath,
              mesh,
              boundary_condition,
              fem_dimensions,
              use_graphics=False,
              ):

    file = open(filepath, "w", encoding="utf8", newline="\n")
    fw = file.write

    has_uv = bool(mesh.uv_textures) and mesh.uv_textures.active
    if has_uv and use_graphics:
        uv_data = mesh.uv_textures.active

    uvcoord = normal = None

    verts = []
    for i, face in enumerate(mesh.polygons):

        if not face.use_smooth:
            normal = face.normal[:]

        if has_uv:
            uv = uv_data[i].uv1, uv_data[i].uv2, uv_data[i].uv3, uv_data[i].uv4

        for j, vert in enumerate(face.vertices):
            v = mesh.vertices[vert]

            if face.use_smooth:
                normal = v.normal[:]

            if has_uv:
                uvcoord = uv[j][0], uv[j][1]

            verts.append((vert, normal, uvcoord))

    elements = []
    if fem_dimensions == '3':
        indices = []
        for face in mesh.polygons:
            print(face.vertices[:])
            new_vert = 0
            for vert in face.vertices:
                if vert not in indices:
                    new_vert += 1

            if new_vert > 0:
                if len(indices) + new_vert <= 4:
                    for vert in face.vertices:
                        if vert not in indices:
                            indices.append(vert)
                else:
                    elements.append(indices)
                    indices = []
                    for vert in face.vertices:
                        indices.append(vert)

        if len(indices) == 4:
            elements.append(indices)

    fw("ply\n")
    fw("format ascii 1.0\n")
    fw("comment Created by Blender %s - "
       "www.blender.org, source file: %r\n" %
       (bpy.app.version_string, os.path.basename(bpy.data.filepath)))

    if fem_dimensions != '1' and use_graphics:
        fw("element vertex %d\n" % len(verts))
    else:
        fw("element vertex %d\n" % len(mesh.vertices))

    fw("property double x\n"
       "property double y\n"
       "property double z\n")

    if use_graphics:
        fw("property double nx\n"
           "property double ny\n"
           "property double nz\n")
        if has_uv:
            fw("property double s\n"
               "property double t\n")

        if fem_dimensions != '1':
            fw("element face %d\n" % len(mesh.polygons))
            fw("property list uint uint vertex_indices\n")
        
    if fem_dimensions == '1':
        fw("element 1d_element %d\n" % (len(mesh.vertices)-1))
        fw("property list uint uint vertex_indices\n")
    elif fem_dimensions == '2':
        fw("element 2d_element %d\n" % len(mesh.polygons))
        fw("property list uint uint vertex_indices\n")
    elif fem_dimensions == '3':
        fw("element 3d_element %d\n" % len(elements))
        fw("property list uint uint vertex_indices\n")

    # TODO Implement physics material property in other addon
    # if passed in variables are empty
    fw("element material 1\n")
    fw("property double mass_density\n")
    fw("property double poisson_ratio\n")
    fw("property double young_modulus\n")

    if boundary_condition is not None:
        fw("element boundary_condition %d\n" % len(boundary_condition))
        fw("property list uint uint vertex_index\n")
    fw("end_header\n")

    if fem_dimensions != '1':
        if use_graphics:
            for vert in verts:
                fw("%.6f %.6f %.6f" % mesh.vertices[vert[0]].co[:])  # co
                fw(" %.6f %.6f %.6f" % vert[1])  # no
                if has_uv:
                    fw(" %.6f %.6f" % vert[2])  # uv
                fw("\n")

            for face in mesh.polygons:
                if len(face.vertices) == 3:
                    fw("3 %d %d %d\n" % face.vertices[:])
                else:
                    fw("4 %d %d %d %d\n" % face.vertices[:])
        else:
            for vert in mesh.vertices:
                fw("%.6f %.6f %.6f\n" % vert.co[:])  # co
    else:
        for vert in mesh.vertices:
            fw("%.6f %.6f %.6f\n" % vert.co[:])  # co
            
    if fem_dimensions is '1':
        for i in (len(mesh.vertices)-1):
            fw("2 %d %d\n" % list(i, i+1))
    elif fem_dimensions is '2':
        for face in mesh.polygons:
            if len(face.vertices) == 3:
                fw("3 %d %d %d\n" % face.vertices[:])
            else:
                fw("3 %d %d %d\n" % face.vertices[:3])
                fw("3 %d %d %d\n" % face.vertices[1:4])
    elif fem_dimensions is '3':
        for tet in elements:
            fw("4 %s\n" % ' '.join(map(str, tet)))

    # TODO Implement physics material property in other addon
    # if passed in variables are empty
    fw("900.0 0.45 1.75e9\n")

    if boundary_condition is not None:
        for bc in boundary_condition:
            fw("%d " % bc)

    file.close()
    print("writing %r done" % filepath)

    return {'FINISHED'}


def menu_func_export(self, context):
    self.layout.operator(ExportPLY.bl_idname, text="OpenSurgSim FEM (.ply)")


def register():
    bpy.utils.register_module(__name__)

    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_module(__name__)

    bpy.types.INFO_MT_file_export.remove(menu_func_export)

if __name__ == "__main__":
    register()
