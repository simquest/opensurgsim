# Copyright 2016 SimQuest Solutions
"""Module to add FEM exporting to Blender"""

from mathutils import Matrix
import bpy
import bmesh
from bpy.props import (StringProperty,
                       BoolProperty,
                       EnumProperty,
                      )
from bpy_extras.io_utils import (ExportHelper)

bl_info = {
    "name": "OpenSurgSim FEM PLY format",
    "author": "Ryan Kornheisl, Paul Novotny",
    "location": "File > Import-Export",
    "description": "Export FEM PLY mesh data for use with OpenSurgSim",
    "support": 'COMMUNITY',
    "category": "Import-Export"}

class ExportPLY(bpy.types.Operator, ExportHelper):
    """Export a single object as a PLY file for use as an FEM in OpenSurgSim"""
    bl_idname = "export_mesh.fem"
    bl_label = "Export FEM for OpenSurgSim"

    filename_ext = ".ply"

    filter_glob = StringProperty(default="*.ply", options={'HIDDEN'})

    use_graphics = BoolProperty(
        name="Export Graphics",
        description="Include data for faces and uv coordinates",
        default=False,
    )
    fem_dimensions = EnumProperty(
        name="Dimension",
        items=(('1', "FEM 1D", ""),
               ('2', "FEM 2D", ""),
               ('3', "FEM 3D", ""),
              ),
        default='2',
    )
    mass_density = StringProperty(
        name="Mass Density",
        default="900.0"
    )
    poisson_ratio = StringProperty(
        name="Poisson Ratio",
        default="0.45"
    )
    young_modulus = StringProperty(
        name="Young Modulus",
        default="1.75e9"
    )
    meshpy = StringProperty(
        name="MeshPy Options",
        default="pq1.5a5MY"
    )

    @classmethod
    def poll(cls, context):
        """Check for active object"""
        return context.active_object is not None

    def execute(self, context):
        """Execute save"""
        keywords = self.as_keywords(ignore=("check_existing",
                                            "filter_glob",
                                           ))

        filepath = self.filepath
        filepath = bpy.path.ensure_ext(filepath, self.filename_ext)

        return export_fem_save(self, context, **keywords)

    def draw(self, context):
        """Creates options UI in save dialog"""
        layout = self.layout

        row = layout.row()
        row.prop(self, "use_graphics")

        layout.prop(self, "fem_dimensions")

        row = layout.row()
        row.prop(self, "mass_density")

        row = layout.row()
        row.prop(self, "poisson_ratio")

        row = layout.row()
        row.prop(self, "young_modulus")

        row = layout.row()
        row.prop(self, "meshpy")


def check_for_holes(mesh):
    """Checks for non-manifold elements (aka holes)

       Return True if there are holes
    """
    if bpy.context.mode != 'EDIT_MESH':
        bpy.ops.object.mode_set(mode='EDIT')

    blender_mesh = bmesh.from_edit_mesh(mesh)
    for elem in getattr(blender_mesh, 'edges'):
        if not elem.is_manifold:
            bpy.ops.object.mode_set(mode='OBJECT')
            return True

    bpy.ops.object.mode_set(mode='OBJECT')
    return False

def export_fem_save(operator,
                    context,
                    use_graphics,
                    fem_dimensions,
                    mass_density,
                    poisson_ratio,
                    young_modulus,
                    meshpy,
                    filepath="",
                   ):
    """Preps a copy of the mesh for exporting"""
    blender_object = context.active_object

    if fem_dimensions == '3':
        if check_for_holes(blender_object.data):
            operator.report({'ERROR'}, "You cannot export an FEM 3D from a mesh that is not closed")
            return {'CANCELLED'}

    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')

    mesh = blender_object.data.copy()

    if not mesh:
        raise Exception("Error, could not get mesh data from active object")

    if fem_dimensions == '1':
        use_graphics = False

    if use_graphics:
        mesh.calc_normals()

    mesh.transform(Matrix() * blender_object.matrix_world)

    ret = save_mesh(filepath, mesh, blender_object.get('boundary_condition'),
                    use_graphics,
                    fem_dimensions,
                    mass_density,
                    poisson_ratio,
                    young_modulus,
                    meshpy,
                   )

    bpy.data.meshes.remove(mesh)

    return ret

def save_mesh(filepath,
              mesh,
              boundary_condition,
              use_graphics,
              fem_dimensions,
              mass_density,
              poisson_ratio,
              young_modulus,
              meshpy
             ):
    """Write mesh data out to ply file

       If graphics are checked to be exported, it will prep any UVs and normals it can and prepare
       a full list of faces FEM3D meshes have their mesh run through MeshPy which is a Python
       interface for TetGen
    """

    file = open(filepath, "w", encoding="utf8", newline="\n")
    fwrite = file.write

    verts = []
    faces = []
    elements = []
    uvcoords = {}
    normals = {}

    has_uv = bool(mesh.uv_textures) and mesh.uv_textures.active
    if has_uv and use_graphics:
        mesh.calc_tessface()
        uv_data = mesh.tessface_uv_textures.active.data
        for i, polygon in enumerate(mesh.polygons):
            uv = uv_data[i].uv1, uv_data[i].uv2, uv_data[i].uv3, uv_data[i].uv4
            for j, vert in enumerate(polygon.vertices):
                uvcoord = uv[j][0], uv[j][1]
                normal = mesh.vertices[vert].normal[:]
                vertcoord = mesh.vertices[vert].co[:]
                if vertcoord not in uvcoords:
                    uvcoords[vertcoord] = uvcoord
                if vertcoord not in normals:
                    normals[vertcoord] = normal

    if fem_dimensions == '3':
        from meshpy.tet import MeshInfo, build, Options

        bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
        bpy.context.scene.update()

        # get list of verts and faces direct from mesh
        mesh_verts = []
        for vert in mesh.vertices:
            mesh_verts.append(vert.co[:])

        mesh_faces = []
        for face in mesh.polygons:
            mesh_faces.append(face.vertices[:])

        args = meshpy
        mesh_info = MeshInfo()
        mesh_info.set_points(mesh_verts)
        mesh_info.set_facets(mesh_faces)
        tets = build(mesh_info, Options(args),
                     verbose=True,
                     attributes=False,
                     volume_constraints=False,
                     max_volume=None,
                     diagnose=False,
                     insert_points=None)

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

    fwrite("ply\n")
    fwrite("format ascii 1.0\n")
    fwrite("comment Created by OpenSurgSim FEM exporter for Blender\n")
    fwrite("MeshPy options '%s'\n" % meshpy)

    fwrite("element vertex %d\n" % len(verts))
    fwrite("property double x\n"
           "property double y\n"
           "property double z\n")

    if use_graphics:
        if fem_dimensions != '1':
            if normals:
                fwrite("property double nx\n"
                       "property double ny\n"
                       "property double nz\n")
            if has_uv:
                fwrite("property double s\n"
                       "property double t\n")
            fwrite("element face %d\n" % len(faces))
            fwrite("property list uint uint vertex_indices\n")

    if fem_dimensions == '1':
        fwrite("element 1d_element %d\n" % len(mesh.vertices)-1)
        fwrite("property list uint uint vertex_indices\n")
    elif fem_dimensions == '2':
        fwrite("element 2d_element %d\n" % len(faces))
        fwrite("property list uint uint vertex_indices\n")
    elif fem_dimensions == '3':
        fwrite("element 3d_element %d\n" % len(elements))
        fwrite("property list uint uint vertex_indices\n")

    fwrite("element material 1\n")
    fwrite("property double mass_density\n")
    fwrite("property double poisson_ratio\n")
    fwrite("property double young_modulus\n")

    if boundary_condition is not None:
        fwrite("element boundary_condition %d\n" % len(boundary_condition))
        fwrite("property uint vertex_index\n")
    fwrite("end_header\n")

    if fem_dimensions != '1':
        if use_graphics:
            for vert in verts:
                fwrite("%.6f %.6f %.6f" % vert[:])
                if vert in normals:
                    fwrite(" %.6f %.6f %.6f" % normals[vert][:])
                if has_uv and vert in uvcoords:
                    fwrite(" %.6f %.6f" % uvcoords[vert][:])
                fwrite("\n")
            for face in faces:
                fwrite("3 %d %d %d\n" % face[:])
        else:
            for vert in verts:
                fwrite("%.6f %.6f %.6f\n" % vert[:])
    else:
        for vert in mesh.vertices:
            fwrite("%.6f %.6f %.6f\n" % vert.co[:])

    if fem_dimensions is '1':
        for i in len(mesh.vertices)-1:
            fwrite("2 %d %d\n" % list(i, i+1))
    elif fem_dimensions is '2':
        for face in faces:
            fwrite("3 %d %d %d\n" % face[:])
    elif fem_dimensions is '3':
        for tet in elements:
            fwrite("4 %d %d %d %d\n" % tet[:])

    fwrite("%s %s %s\n" % (mass_density, poisson_ratio, young_modulus))

    if boundary_condition is not None:
        for index in boundary_condition:
            fwrite("%d\n" % index)

    file.close()
    print("writing %r done" % filepath)

    return {'FINISHED'}


def menu_func_export(self, context):
    """Adds menu item to Blender"""
    self.layout.operator(ExportPLY.bl_idname, text="OpenSurgSim FEM (.ply)")


def register():
    """Registers addon with Blender"""
    bpy.utils.register_module(__name__)

    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    """Unregisters addon with Blender"""
    bpy.utils.unregister_module(__name__)

    bpy.types.INFO_MT_file_export.remove(menu_func_export)

if __name__ == "__main__":
    register()
