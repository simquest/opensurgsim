# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

bl_info = {
    "name": "OpenSurgSim FEM PLY format",
    "author": "Ryan Kornheisl, Paul Novotny",
    "location": "File > Import-Export",
    "description": "Export FEM PLY mesh data for use with OpenSurgSim",
    "support": 'COMMUNITY',
    "category": "Import-Export"}

# Based on Stanford PLY exporter from Bruce Merry

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
    bl_label = "Export FEM for OSS"

    filename_ext = ".ply"
    filter_glob = StringProperty(default="*.ply", options={'HIDDEN'})

    use_mesh_modifiers = BoolProperty(
            name="Apply Modifiers",
            description="Apply Modifiers to the exported mesh",
            default=True,
            )
    use_normals = BoolProperty(
            name="Normals",
            description="Export Normals for smooth and "
                        "hard shaded faces "
                        "(hard shaded faces will be exported "
                        "as individual faces)",
            default=True,
            )
    use_uv_coords = BoolProperty(
            name="UVs",
            description="Export the active UV layer",
            default=True,
            )
    use_colors = BoolProperty(
            name="Vertex Colors",
            description="Export the active vertex color layer",
            default=True,
            )

    axis_forward = EnumProperty(
            name="Forward",
            items=(('X', "X Forward", ""),
                   ('Y', "Y Forward", ""),
                   ('Z', "Z Forward", ""),
                   ('-X', "-X Forward", ""),
                   ('-Y', "-Y Forward", ""),
                   ('-Z', "-Z Forward", ""),
                   ),
            default='Y',
            )
    axis_up = EnumProperty(
            name="Up",
            items=(('X', "X Up", ""),
                   ('Y', "Y Up", ""),
                   ('Z', "Z Up", ""),
                   ('-X', "-X Up", ""),
                   ('-Y', "-Y Up", ""),
                   ('-Z', "-Z Up", ""),
                   ),
            default='Z',
            )
    global_scale = FloatProperty(
            name="Scale",
            min=0.01, max=1000.0,
            default=1.0,
            )

    @classmethod
    def poll(cls, context):
        return context.active_object != None

    def execute(self, context):
        from mathutils import Matrix

        keywords = self.as_keywords(ignore=("axis_forward",
                                            "axis_up",
                                            "global_scale",
                                            "check_existing",
                                            "filter_glob",
                                            ))
        global_matrix = axis_conversion(to_forward=self.axis_forward,
                                        to_up=self.axis_up,
                                        ).to_4x4() * Matrix.Scale(self.global_scale, 4)
        keywords["global_matrix"] = global_matrix

        filepath = self.filepath
        filepath = bpy.path.ensure_ext(filepath, self.filename_ext)

        return export_fem_save(self, context, **keywords)

    def draw(self, context):
        layout = self.layout

        row = layout.row()
        row.prop(self, "use_mesh_modifiers")
        row.prop(self, "use_normals")
        row = layout.row()
        row.prop(self, "use_uv_coords")
        row.prop(self, "use_colors")

        layout.prop(self, "axis_forward")
        layout.prop(self, "axis_up")
        layout.prop(self, "global_scale")


def export_fem_save(operator,
         context,
         filepath="",
         use_mesh_modifiers=True,
         use_normals=True,
         use_uv_coords=True,
         use_colors=True,
         global_matrix=None
         ):

    scene = context.scene
    obj = context.active_object

    if global_matrix is None:
        from mathutils import Matrix
        global_matrix = Matrix()

    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')

    if use_mesh_modifiers and obj.modifiers:
        mesh = obj.to_mesh(scene, True, 'PREVIEW')
    else:
        mesh = obj.data.copy()

    if not mesh:
        raise Exception("Error, could not get mesh data from active object")

    mesh.transform(global_matrix * obj.matrix_world)
    if use_normals:
        mesh.calc_normals()

    ret = save_mesh(filepath, mesh, obj.get('boundary_condition'),
                    use_normals=use_normals,
                    use_uv_coords=use_uv_coords,
                    use_colors=use_colors,
                    )

    if use_mesh_modifiers:
        bpy.data.meshes.remove(mesh)

    return ret


def save_mesh(filepath,
              mesh,
              boundary_condition,
              use_normals=True,
              use_uv_coords=True,
              use_colors=True,
              ):

    def rvec3d(v):
        return round(v[0], 6), round(v[1], 6), round(v[2], 6)

    def rvec2d(v):
        return round(v[0], 6), round(v[1], 6)

    file = open(filepath, "w", encoding="utf8", newline="\n")
    fw = file.write

    # Be sure tessface & co are available!
    if not mesh.tessfaces and mesh.polygons:
        mesh.calc_tessface()

    has_uv = bool(mesh.tessface_uv_textures)
    has_vcol = bool(mesh.tessface_vertex_colors)

    if not has_uv:
        use_uv_coords = False
    if not has_vcol:
        use_colors = False

    if not use_uv_coords:
        has_uv = False
    if not use_colors:
        has_vcol = False

    if has_uv:
        active_uv_layer = mesh.tessface_uv_textures.active
        if not active_uv_layer:
            use_uv_coords = False
            has_uv = False
        else:
            active_uv_layer = active_uv_layer.data

    if has_vcol:
        active_col_layer = mesh.tessface_vertex_colors.active
        if not active_col_layer:
            use_colors = False
            has_vcol = False
        else:
            active_col_layer = active_col_layer.data

    # in case
    color = uvcoord = uvcoord_key = normal = normal_key = None

    mesh_verts = mesh.vertices  # save a lookup
    ply_verts = []  # list of dictionaries
    # vdict = {} # (index, normal, uv) -> new index
    vdict = [{} for i in range(len(mesh_verts))]
    ply_faces = [[] for f in range(len(mesh.tessfaces))]
    vert_count = 0
    for i, f in enumerate(mesh.tessfaces):

        smooth = not use_normals or f.use_smooth
        if not smooth:
            normal = f.normal[:]
            normal_key = rvec3d(normal)

        if has_uv:
            uv = active_uv_layer[i]
            uv = uv.uv1, uv.uv2, uv.uv3, uv.uv4
        if has_vcol:
            col = active_col_layer[i]
            col = col.color1[:], col.color2[:], col.color3[:], col.color4[:]

        f_verts = f.vertices

        pf = ply_faces[i]
        for j, vidx in enumerate(f_verts):
            v = mesh_verts[vidx]

            if smooth:
                normal = v.normal[:]
                normal_key = rvec3d(normal)

            if has_uv:
                uvcoord = uv[j][0], uv[j][1]
                uvcoord_key = rvec2d(uvcoord)

            if has_vcol:
                color = col[j]
                color = (int(color[0] * 255.0),
                         int(color[1] * 255.0),
                         int(color[2] * 255.0),
                         )
            key = normal_key, uvcoord_key, color

            vdict_local = vdict[vidx]
            pf_vidx = vdict_local.get(key)  # Will be None initially

            if pf_vidx is None:  # same as vdict_local.has_key(key)
                pf_vidx = vdict_local[key] = vert_count
                ply_verts.append((vidx, normal, uvcoord, color))
                vert_count += 1

            pf.append(pf_vidx)

    fw("ply\n")
    fw("format ascii 1.0\n")
    fw("comment Created by Blender %s - "
       "www.blender.org, source file: %r\n" %
       (bpy.app.version_string, os.path.basename(bpy.data.filepath)))

    fw("element vertex %d\n" % len(ply_verts))

    fw("property float x\n"
       "property float y\n"
       "property float z\n")

    if use_normals:
        fw("property float nx\n"
           "property float ny\n"
           "property float nz\n")
    if use_uv_coords:
        fw("property float s\n"
           "property float t\n")
    if use_colors:
        fw("property uchar red\n"
           "property uchar green\n"
           "property uchar blue\n")

    fw("element face %d\n" % len(mesh.tessfaces))
    fw("property list uchar uint vertex_indices\n")

    if boundary_condition is not None:
        fw("element boundary_condition %d\n" % len(boundary_condition))
        fw("property list uint uint vertex_index\n")
    fw("end_header\n")

    for i, v in enumerate(ply_verts):
        fw("%.6f %.6f %.6f" % mesh_verts[v[0]].co[:])  # co
        if use_normals:
            fw(" %.6f %.6f %.6f" % v[1])  # no
        if use_uv_coords:
            fw(" %.6f %.6f" % v[2])  # uv
        if use_colors:
            fw(" %u %u %u" % v[3])  # col
        fw("\n")

    for pf in ply_faces:
        if len(pf) == 3:
            fw("3 %d %d %d\n" % tuple(pf))
        else:
            fw("4 %d %d %d %d\n" % tuple(pf))

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
