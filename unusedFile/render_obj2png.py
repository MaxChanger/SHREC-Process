import bpy
import pathlib

# Adjust this for where you have the OBJ files.
obj_root = pathlib.Path('bad_model_obj')

# Before we start, make sure nothing is selected. The importer will select
# imported objects, which allows us to delete them after rendering.
bpy.ops.object.select_all(action='DESELECT')
render = bpy.context.scene.render

for obj_fname in obj_root.glob('*.obj'):
    bpy.ops.import_scene.obj(filepath=str(obj_fname))

    render.filepath = '//obj-%s' % obj_fname.stem
    bpy.ops.render.render(write_still=True)

    # Remember which meshes were just imported
    meshes_to_remove = []
    for ob in bpy.context.selected_objects:
        meshes_to_remove.append(ob.data)

    bpy.ops.object.delete()

    # Remove the meshes from memory too
    for mesh in meshes_to_remove:
        bpy.data.meshes.remove(mesh)