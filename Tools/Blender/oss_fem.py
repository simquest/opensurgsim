bl_info = {
    "name": "OpenSurgSim Boundary Condition",
    "description": "Script to allow adding boundary condition property to vertices",
    "author": "Ryan Kornheisl, Paul Novotny",
    "support": "COMMUNITY",
    "category": "Mesh",
}

import bpy
import bmesh


class AddBoundaryCondition(bpy.types.Operator):
    """Add boundary condition to selected vertices"""
    bl_idname = "mesh.boundary_condition"
    bl_label = "Add Boundary Condition"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        if context.object.mode == 'EDIT':
            verts = [i.index for i in bmesh.from_edit_mesh(bpy.context.active_object.data).verts if i.select]
            if len(verts) > 0:
                context.object["boundary_condition"] = verts
                
        return {'FINISHED'}

def menu_func(self,context):
    self.layout.operator(AddBoundaryCondition.bl_idname)

def register():
    bpy.utils.register_class(AddBoundaryCondition)
    bpy.types.VIEW3D_MT_edit_mesh.append(menu_func)


def unregister():
    bpy.utils.unregister_class(AddBoundaryCondition)
    bpy.types.VIEW3D_MT_edit_mesh.remove(menu_func)


# This allows you to run the script directly from blenders text editor
# to test the addon without having to install it.
if __name__ == "__main__":
    register()
