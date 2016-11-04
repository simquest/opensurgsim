# Copyright 2016 SimQuest Solutions

bl_info = {
    "name": "OpenSurgSim FEM PLY format",
    "author": "Ryan Kornheisl, Paul Novotny",
    "location": "File > Import-Export",
    "description": "Export FEM PLY mesh data for use with OpenSurgSim",
    "support": 'COMMUNITY',
    "category": "Import-Export"}

import os
from mathutils import Matrix
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

    mesh.transform(Matrix() * object.matrix_world)

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

    verts = []
    faces = []
    elements = []

    if fem_dimensions == '3':
        from meshpy.tet import MeshInfo, build, Options

        bpy.ops.object.mode_set(mode = 'OBJECT', toggle = False)
        bpy.context.scene.update()

        # get list of verts and faces direct from mesh
        mesh_verts = []
        for vert in mesh.vertices:
            mesh_verts.append(vert.co[:])

        mesh_faces = []
        for face in mesh.polygons:
            mesh_faces.append(face.vertices[:])

        args = "pq1.5a5MY";
        mesh_info = MeshInfo()
        mesh_info.set_points(mesh_verts)
        mesh_info.set_facets(mesh_faces)
        tets = build(mesh_info, Options(args),
                     verbose = True,
                     attributes = False,
                     volume_constraints = False,
                     max_volume = None,
                     diagnose = False,
                     insert_points = None)

        # Ply prep by creating updated vert list and face list using tets.points and tets.elements
        for point in tets.points:
            verts.append((point[0], point[1], point[2]))

        for tet in tets.elements:
            elements.append((tet[0], tet[1], tet[2], tet[3]))

        for face in tets.faces:
            faces.append((face[0], face[1], face[2]))

    else:
        for vert in mesh.vertices:
            verts.append((vert.co[0], vert.co[1], vert.co[2]))

        for face in mesh.polygons:
            if len(face.vertices) == 3:
                faces.append((face.vertices[0], face.vertices[1], face.vertices[2]))
            else:
                faces.append((face.vertices[0], face.vertices[1], face.vertices[2]))
                faces.append((face.vertices[0], face.vertices[2], face.vertices[3]))

    fw("ply\n")
    fw("format ascii 1.0\n")
    fw("comment Created by Blender %s - "
       "www.blender.org, source file: %r\n" %
       (bpy.app.version_string, os.path.basename(bpy.data.filepath)))

    fw("element vertex %d\n" % len(verts))
    fw("property double x\n"
       "property double y\n"
       "property double z\n")

    if use_graphics:
        if fem_dimensions != '1':
            fw("element face %d\n" % len(faces))
            fw("property list uint uint vertex_indices\n")

    if fem_dimensions == '1':
        fw("element 1d_element %d\n" % vert_count-1)
        fw("property list uint uint vertex_indices\n")
    elif fem_dimensions == '2':
        fw("element 2d_element %d\n" % len(faces))
        fw("property list uint uint vertex_indices\n")
    elif fem_dimensions == '3':
        fw("element 3d_element %d\n" % len(elements))
        fw("property list uint uint vertex_indices\n")

    if boundary_condition is not None:
        fw("element boundary_condition %d\n" % len(boundary_condition))
        fw("property uint vertex_index\n")
    fw("end_header\n")

    if fem_dimensions != '1':
        if use_graphics:
            for vert in verts:
                fw("%.6f %.6f %.6f\n" % vert[:])
            for face in faces:
                if len(face) == 3:
                    fw("3 %d %d %d\n" % face[:])
                else:
                    fw("4 %d %d %d %d\n" % face[:])
        else:
            for vert in verts:
                fw("%.6f %.6f %.6f" % vert[:])
    else:
        for vert in mesh.vertices:
            fw("%.6f %.6f %.6f\n" % vert.co[:])

    if fem_dimensions is '1':
        for i in (len(mesh.vertices)-1):
            fw("2 %d %d\n" % list(i, i+1))
    elif fem_dimensions is '2':
        for face in faces:
            fw("3 %d %d %d\n" % face[:])
    elif fem_dimensions is '3':
        for tet in elements:
            fw("4 %d %d %d %d\n" % tet[:])

    if boundary_condition is not None:
        for bc in boundary_condition:
            fw("%d\n" % bc)

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
