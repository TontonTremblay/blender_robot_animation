import sys
with open("path.txt") as file:
    paths = eval(file.read())
for p in paths:
    sys.path.insert(0,p)
import argparse, sys, os, math, re
import bpy
from mathutils import Vector, Matrix
import mathutils
import numpy as np
import json 
import random 
import glob 
import threading


from yourdfpy import URDF
import yourdfpy
import random 
import pyrr 
import scipy 
from utils import * 
import argparse
import yaml

from bpy_extras.image_utils import load_image
from omegaconf import OmegaConf


parser = argparse.ArgumentParser(description='Renders given obj file by rotation a camera around it.')
parser.add_argument(
    '--config', default=None,
    help='skip the scene set up')

argv = sys.argv[sys.argv.index("--") + 1:]
opt = parser.parse_args(argv)

config_file = OmegaConf.load(opt.config)



def is_image_loaded(image_filepath):
    for image in bpy.data.images:
        if image.filepath == image_filepath:
            return image
    return None


def add_material(obj,path_texture, add_uv = False, material_pos = -1):
    if add_uv:
        bpy.ops.object.select_all(action='DESELECT')
        bpy.context.view_layer.objects.active = obj

        obj.select_set(True)

        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')

        # Unwrap using Smart UV Project
        bpy.ops.uv.smart_project()
        # bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0.001)

        # Switch back to Object mode
        bpy.ops.object.mode_set(mode='OBJECT')

    name = path_texture.split("/")[-2]
    bpy.context.view_layer.objects.active = obj

    # Create a new material
    new_material = bpy.data.materials.new(name=f"{obj.name}_Material")
    
    # Link the material to the object
    # print(len(obj.data.materials))
    if material_pos == -1 or len(obj.data.materials) ==0:
        obj.data.materials.clear()

        obj.data.materials.append(new_material)
    else:
        obj.data.materials[material_pos] = new_material

    new_material.use_nodes = True
    node_tree = new_material.node_tree

    # Clear default nodes
    for node in node_tree.nodes:
        node_tree.nodes.remove(node)

    principled_node = node_tree.nodes.new(type='ShaderNodeBsdfPrincipled')

    # Create an image texture node
    image_texture_node = node_tree.nodes.new(type='ShaderNodeTexImage')
    
    image = is_image_loaded(f"{path_texture}/{name}_2K-JPG_Color.jpg")
    if image is None:
        image = load_image(f"{path_texture}/{name}_2K-JPG_Color.jpg", new_material)
    image_texture_node.image = image

    # # normal
    img_normal = is_image_loaded(f"{path_texture}/{name}_2K-JPG_NormalGL.jpg")
    if img_normal is None:
        img_normal = load_image(f"{path_texture}/{name}_2K-JPG_NormalGL.jpg", new_material)
    image_texture_node_normal = node_tree.nodes.new(type='ShaderNodeTexImage')
    image_texture_node_normal.image = img_normal    
    image_texture_node_normal.image.colorspace_settings.name = 'Non-Color'

    normal_map_node = node_tree.nodes.new(type='ShaderNodeNormalMap')

    node_tree.links.new(normal_map_node.outputs["Normal"], principled_node.inputs["Normal"])
    node_tree.links.new(image_texture_node_normal.outputs["Color"], normal_map_node.inputs["Color"])


    # rough
    if os.path.exists(f"{path_texture}/{name}_2K-JPG_Roughness.jpg"):
        img_rough = is_image_loaded(f"{path_texture}/{name}_2K-JPG_Roughness.jpg")
        if img_rough is None:
            img_rough = load_image(f"{path_texture}/{name}_2K-JPG_Roughness.jpg", new_material)

        image_texture_node_rough = node_tree.nodes.new(type='ShaderNodeTexImage')
        image_texture_node_rough.image = img_rough    
        image_texture_node_rough.image.colorspace_settings.name = 'Non-Color'

        node_tree.links.new(image_texture_node_rough.outputs["Color"], principled_node.inputs["Roughness"])

    # metal
    if os.path.exists(f"{path_texture}/{name}_2K-JPG_Metalness.jpg"):
        img_metal = is_image_loaded(f"{path_texture}/{name}_2K-JPG_Metalness.jpg")
        if img_metal is None:
            img_metal = load_image(f"{path_texture}/{name}_2K-JPG_Metalness.jpg", new_material)

        image_texture_node_metal = node_tree.nodes.new(type='ShaderNodeTexImage')
        image_texture_node_metal.image = img_metal    
        image_texture_node_metal.image.colorspace_settings.name = 'Non-Color'

        node_tree.links.new(image_texture_node_metal.outputs["Color"], principled_node.inputs["Metallic"])



    # connecting
    node_tree.links.new(image_texture_node.outputs["Color"], principled_node.inputs["Base Color"])
    
    material_output_node = node_tree.nodes.new(type='ShaderNodeOutputMaterial')
    node_tree.links.new(principled_node.outputs["BSDF"], material_output_node.inputs["Surface"])


def sample_points_on_sphere(num_points, elevation_range, theta_range):
    """
    Sample points on a sphere given elevation and azimuth (theta) ranges.

    Parameters:
    - num_points: Number of points to sample.
    - elevation_range: Tuple containing the minimum and maximum elevation angles in radians.
    - theta_range: Tuple containing the minimum and maximum azimuth angles in radians.

    Returns:
    - points: Array of sampled points in Cartesian coordinates.
    """

    # elev_min, elev_max = elevation_range
    # theta_min, theta_max = theta_range

    elev_min, elev_max = np.radians(elevation_range)
    theta_min, theta_max = np.radians(theta_range)


    elev = np.random.uniform(elev_min, elev_max, num_points)
    theta = np.random.uniform(theta_min, theta_max, num_points)

    x = np.sin(elev) * np.cos(theta)
    y = np.sin(elev) * np.sin(theta)
    z = np.cos(elev)

    points = np.column_stack((x, y, z))
    return points



def delete_all_cameras():
    # Get all objects in the scene
    all_objects = bpy.data.objects
    
    # Iterate through each object
    for obj in all_objects:
        # Check if the object is a camera
        if obj.type == 'CAMERA':
            # Remove the camera object
            bpy.data.objects.remove(obj, do_unlink=True)

def delete_all_lights():
    # Get all objects in the scene
    all_objects = bpy.data.objects
    
    # Iterate through each object
    for obj in all_objects:
        # Check if the object is a light
        if obj.type == 'LIGHT':
            # Remove the light object
            bpy.data.objects.remove(obj, do_unlink=True)


##### CLEAN BLENDER SCENES ##### 
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

RESOLUTION = 512
##### SET THE RENDERER ######
render = bpy.context.scene.render
render.engine = "CYCLES"
render.image_settings.color_mode = 'RGBA'  # ('RGB', 'RGBA', ...)
render.image_settings.file_format = 'PNG'  # ('PNG', 'OPEN_EXR', 'JPEG, ...)
render.resolution_x = RESOLUTION
render.resolution_y = RESOLUTION
render.resolution_percentage = 100
bpy.context.scene.cycles.filter_width = 0.01
# bpy.context.scene.render.film_transparent = True

bpy.context.scene.cycles.device = 'GPU'
bpy.context.scene.cycles.diffuse_bounces = 1
bpy.context.scene.cycles.glossy_bounces = 1
bpy.context.scene.cycles.transparent_max_bounces = 3
bpy.context.scene.cycles.transmission_bounces = 3
bpy.context.scene.cycles.samples = 32
bpy.context.scene.cycles.use_denoising = True


##### LOAD THE URDF SCENE #####
# urdf_content_path = "/Users/jtremblay/code/yourdfpy/robot/kitchen_urdf/"
# kitchen = URDF.load(f"{urdf_content_path}/kitchen.urdf")

# JACO
urdf_content_path = config_file.content.urdf_content_path

kitchen = URDF.load(f"{urdf_content_path}/{config_file.content.urdf_path}")

# reachy
# urdf_content_path = "/Users/jtremblay/code/blender_robot_animation/models/reachy_description/"
# kitchen = URDF.load(f"{urdf_content_path}/reachy.URDF")


s = config_file.seed
random.seed(s)
np.random.seed(s)

NB_FRAMES = config_file.render.nb_frames

cfg_start = {}
data_structure = {}

children_structure = {}
parent_structure = {}

link_top2joint = {}

for joint_name in kitchen.joint_names:
    j = kitchen.joint_map[joint_name]
    if not j.type == 'fixed':
        pass
        # print(joint_name,j.type,j.origin)
        # print(joint_name,j.type)
    else:
        print(joint_name,j.type)

    data_structure[joint_name] = {
        'parent':j.parent,
        'child':j.child
    }
    link_top2joint[j.child] = joint_name

    if j.parent in children_structure:
        children_structure[j.parent].append(j.child)
    else:
        children_structure[j.parent]=[j.child]
    parent_structure[j.child]=j.parent
    # if 'range' in j.child:
        # print(j.child,'is child to',j.parent)
    if not j.limit is None: 
        # print(j.limit)
        cfg_start[joint_name] = random.uniform(j.limit.lower,j.limit.upper)



# raise()
# print(data_structure)

#make the interpret
values_to_render = {}

for joint_name in cfg_start.keys():
    # print(joint_name)
    j = kitchen.joint_map[joint_name]
    # print(j.origin)
    nb_poses = config_file.content.nb_poses
    x = np.linspace(0,1,nb_poses)
    y = [random.uniform(j.limit.lower,j.limit.upper) for i in range(nb_poses)]
    for iv in range(len(y)):
        if random.uniform(0,1)>0.6 :
            y[iv] = j.limit.lower
    inter = scipy.interpolate.interp1d(x,y,
        # kind= random.choice(['linear', 'quadratic', 'cubic']),
        kind= random.choice(['linear']),
    )
    # else:
    # inter = scipy.interpolate.splrep(x,y,
    #     # kind= random.choice(['linear', 'quadratic', 'cubic']),
    #     )


    x_values = np.linspace(0,1,NB_FRAMES)
    
    values = inter(x_values)
    values[values < j.limit.lower] = j.limit.lower
    values[values > j.limit.upper] = j.limit.upper

    values_to_render[joint_name] = values
# raise()
bpy.context.scene.frame_end=NB_FRAMES

kitchen.update_cfg(cfg_start)

# load in blender. 
link2blender = {}
DATA_2_EXPORT = {}

bpy.ops.object.select_all(action='DESELECT')

root_obj = None


        # if root_obj is None:
        #     root_obj = ob
all_textures = glob.glob(config_file.content.materials_path+"*/")
origin_trans = {}

for link in kitchen.link_map.keys():    
    link_name = link
    link = kitchen.link_map[link]
    # print(link_name,link.parent,link.child)


    if len(link.visuals) == 0: 
        bpy.ops.object.empty_add(radius=0.05,location=(0,0,0))
        obj = bpy.context.object
        obj.name = link_name
        link2blender[link_name] = obj
        add_annotation(obj,empty=True,link_name=link_name, data_parent=parent_structure)

    for visual in link.visuals:

        path = visual.geometry.mesh.filename.replace("package://","")
        path = path.replace("reachy_description/","")
        path = path.replace("baxter_description/","")
        if 'allegro' in path:
            path = path + "/meshes/"

        file_type = visual.geometry.mesh.filename.split('.')[-1]

        bpy.ops.object.select_all(action='DESELECT')

        if not visual.geometry.mesh is None:

            data_2_load = os.path.join(f'{urdf_content_path}/',path)
            print(data_2_load)
            # bpy.ops.import_scene.obj(filepath=data_2_load)
            if file_type.lower() == 'obj':
                bpy.ops.wm.obj_import(filepath=data_2_load)
            elif file_type.lower() == 'dae':
                bpy.ops.wm.collada_import(filepath=data_2_load)
            elif file_type.lower() == 'stl':
                bpy.ops.wm.stl_import(filepath=data_2_load)

            delete_all_lights()
            delete_all_cameras()

            print(visual.origin)


            selected_objects = bpy.context.selected_objects

            # Deselect all objects
            bpy.ops.object.select_all(action='DESELECT')

            # Select mesh objects among the selected objects
            for obj in selected_objects:
                if obj.type == 'MESH':
                    print(obj.name)
                    obj.select_set(True)
            # Join selected mesh objects
            bpy.ops.object.join()

            bpy.ops.object.shade_smooth(use_auto_smooth=True)
                    
            obj = bpy.context.selected_objects[0]
            obj.name = link_name
            bpy.context.view_layer.objects.active = obj



            # bpy.ops.object.select_all(action='DESELECT')

            # for ob in selected_objects:
            #     try:
            #         if (ob.name in bpy.data.objects) and not ob.type == 'MESH':
            #             ob.select_set(True)
            #     except: 
            #         pass

            # bpy.ops.object.delete()


            # bpy.ops.wm.save_as_mainfile(filepath=f"/Users/jtremblay/code/blender_robot_animation/urdf.blend")



            # bpy.ops.object.select_all(action='DESELECT')

            # bpy.ops.object.mode_set(mode='EDIT')
            # # # Select the geometry
            # bpy.ops.mesh.select_all(action='SELECT')
            # # # Call the smart project operator
            # bpy.ops.uv.smart_project()
            # # # Toggle out of Edit Mode
            # bpy.ops.object.mode_set(mode='OBJECT')

            # bpy.ops.object.shade_smooth(use_auto_smooth=True)
            
            path = all_textures[np.random.randint(0,len(all_textures)-1)]
            print(path)
            add_material(obj,path)


            link2blender[link_name] = obj

            # raise()
        elif not visual.geometry.box is None:
            s = visual.geometry.box.size
            s = (s[0]/2,s[1]/2,s[2]/2)
            bpy.ops.mesh.primitive_cube_add()
            obj = bpy.context.active_object
            obj.scale = s
            bpy.ops.object.transform_apply(scale=True)
            obj.name = link_name
            link2blender[link_name] = obj

        elif not visual.geometry.cylinder is None:
            cyl = visual.geometry.cylinder
            bpy.ops.mesh.primitive_cylinder_add(radius=cyl.radius,depth=cyl.length)
            obj = bpy.context.active_object
            obj.name = link_name
            link2blender[link_name] = obj
        else:
            print(visual.geometry, 'not supported')

        if not visual.origin is None:
            # reset transformation
            # bpy.ops.object.select_all(action='DESELECT')
            # obj.select_set(True)
            # bpy.context.view_layer.objects.active = obj
            origin_trans[link_name] = visual.origin

        #### ADD ANNOTATION ####
        add_annotation(obj,link_name= link_name,data_parent=parent_structure)

        DATA_2_EXPORT[obj.name] = {}

    # if not link.inertial is None and not link.inertial.origin is None:  
    #     origin_trans[link_name] = link.inertial.origin


    if link_name not in parent_structure.keys():
        root_obj = obj
        bpy.ops.wm.save_as_mainfile(filepath=f"/Users/jtremblay/code/blender_robot_animation/urdf.blend")

        # break

bpy.context.view_layer.update()



###### UPDATE THE KITCHEN to 0,0,0
cfg = {}
for link in values_to_render.keys():
    cfg[link] = values_to_render[link][0]

kitchen.update_cfg(cfg)

for link in link2blender.keys():

    trans = kitchen.get_transform(link)
    # print(trans)
    obj = link2blender[link]
    
    obj.location.x = trans[0][-1]
    obj.location.y = trans[1][-1]
    obj.location.z = trans[2][-1]

    # print(trans)

    matrix = pyrr.Matrix44(trans).matrix33
    # print(matrix)
    # matrix = matrix * pyrr.Matrix33.from_x_rotation(-np.pi/2)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion.x = matrix.quaternion.x
    obj.rotation_quaternion.y = matrix.quaternion.y
    obj.rotation_quaternion.z = matrix.quaternion.z
    obj.rotation_quaternion.w = matrix.quaternion.w

bpy.context.view_layer.update()

# raise()

k0 = list(values_to_render.keys())[0]
for i in range(len(values_to_render[k0])):
    cfg = {}
    for link in values_to_render.keys():
        cfg[link] = values_to_render[link][i]

    kitchen.update_cfg(cfg)

    for link in link2blender.keys():
        trans = kitchen.get_transform(link)

        if link in link_top2joint.keys():
            joint=link_top2joint[link]
            joint_origin = kitchen.joint_map[joint].origin

            if link in origin_trans:
                trans = np.dot(trans,np.dot(joint_origin,origin_trans[link]))
            else:
                trans = np.dot(trans,joint_origin)

        elif link in origin_trans:
            trans = np.dot(trans,origin_trans[link])

        obj = link2blender[link]
        
        obj.location.x = trans[0][-1]
        obj.location.y = trans[1][-1]
        obj.location.z = trans[2][-1]

        # print(trans)

        matrix = pyrr.Matrix44(trans).matrix33
        # print(matrix)
        # matrix = matrix * pyrr.Matrix33.from_x_rotation(-np.pi/2)
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion.x = matrix.quaternion.x
        obj.rotation_quaternion.y = matrix.quaternion.y
        obj.rotation_quaternion.z = matrix.quaternion.z
        obj.rotation_quaternion.w = matrix.quaternion.w

        obj.keyframe_insert(data_path='location', frame=i)
        obj.keyframe_insert(data_path='rotation_quaternion', frame=i)


#####################
#####################
#####################
#####################
#####################


# # Parameters
# grid_size = 20  # Size of the grid (5x5 in this case)
# cube_spacing = 10  # Spacing between cubes
# max_random_height = 1.5  # Maximum random height factor
# power_factor = 1.1  # Power factor for the height calculation
# nb_random_textures = 10 
# textures = []
# import glob 
# all_textures = glob.glob(config_file.content.materials_path+"*/")
# for i in range(nb_random_textures):
#     textures.append(all_textures[np.random.randint(0,len(all_textures)-1)])


# # Create cubes
# for x in range(-grid_size // 2, grid_size // 2 + 1):
#     for y in range(-grid_size // 2, grid_size // 2 + 1):
#         # Calculate height based on distance from center
#         distance = max(abs(x), abs(y))
#         height_factor = 1.0 + (max_random_height * random.random() * math.pow(distance, power_factor))

#         bpy.ops.mesh.primitive_cube_add(size=2, location=(x * cube_spacing, y * cube_spacing, 0))
#         cube = bpy.context.object
#         # Apply height factor to the cube
#         cube.scale.x = cube_spacing/2
#         cube.scale.y = cube_spacing/2
#         cube.scale.z *= height_factor
#         cube.location.z = -2.0
#         add_material(cube,textures[np.random.randint(0,len(textures)-1)])

#####################
#####################
#####################
#####################
#####################


# HDR LIGHT
world = bpy.data.worlds['World']
world.use_nodes = True
bg = world.node_tree.nodes['Background']

node_environment = world.node_tree.nodes.new('ShaderNodeTexEnvironment')
# Load and assign the image to the node property
skyboxes = glob.glob(f'{config_file.content.hdri_env_map}*.hdr')
skybox_random_selection = skyboxes[random.randint(0,len(skyboxes)-1)]

node_environment.image = bpy.data.images.load(skybox_random_selection) # Relative path
# node_environment.location = -300,0

world.node_tree.links.new(node_environment.outputs["Color"], bg.inputs["Color"])

# add a camera 
bpy.ops.object.camera_add(location=(0, 0, 0))  # Set the initial location of the camera
camera_object = bpy.context.object
camera_object.name = 'Camera'  # Rename the camera object to 'Camera'
bpy.context.scene.camera = camera_object  # Set the scene's active camera to the newly added camera

track_to_constraint = camera_object.constraints.new(type='TRACK_TO')
track_to_constraint.target = root_obj

# get 4 positions 
nb_anchors = config_file.render.camera.nb_anchors
neg = 1

interval_theta = config_file.render.camera.theta

for i in range(nb_anchors): 

    an = sample_points_on_sphere(
                        num_points = 1,
                        elevation_range = config_file.render.camera.elevation,
                        # elevation_range = [
                        #     i*360//(nb_anchors-1),
                        #     (i+1)*360//(nb_anchors-1)],
                        theta_range = [i*interval_theta//(nb_anchors-1),(i+1)*interval_theta//(nb_anchors-1)]
                    )   

    # an = random_sample_sphere(
    #                     nb_frames = 1,
    #                     elevation_range = [90,120],
    #                     # elevation_range = [
    #                     #     i*360//(nb_anchors-1),
    #                     #     (i+1)*360//(nb_anchors-1)],
    #                     tetha_range = [i*360//(nb_anchors-1),(i+1)*360//(nb_anchors-1)]
    #                 )   


    print(an)
    camera_object.location.x = an[0][0]*config_file.render.camera.distance_center
    camera_object.location.y = an[0][1]*config_file.render.camera.distance_center
    camera_object.location.z = an[0][2]*config_file.render.camera.distance_center 
    # camera_object.location.z = an[0][2]*4 + 0.5 * neg




    camera_object.keyframe_insert(data_path='location', frame=int((i)*(NB_FRAMES//(nb_anchors-1))))


##### CREATE A new scene for segmentation rendering 

########################
########################
########################
########################
########################
########################

make_segmentation_scene()

# bpy.ops.scene.new(type='FULL_COPY')
# bpy.context.scene.name = "segmentation"


# # lets update all the materials of the objects to emmissive
# to_change = []
# for ob in bpy.context.scene.objects:
#     if ob.type == 'MESH':
#         to_change.append(ob)

# import colorsys

# for ob in to_change:
#     while True:
#         c = colorsys.hsv_to_rgb(
#             random.uniform(0,255)/255, 
#             random.uniform(200,255)/255, 
#             random.uniform(200,255)/255
#             )
#         found = False
#         for obj in DATA_2_EXPORT:
#             if 'color_seg' in DATA_2_EXPORT[obj] and c == DATA_2_EXPORT[obj]['color_seg']:
#                 found = True
#         if found is True:
#             continue
#         if ob.name.split(".")[0] in DATA_2_EXPORT:
#             DATA_2_EXPORT[ob.name.split(".")[0]]['color_seg'] = c
#         break
#     EmissionColorObj(ob,c)

# nodes = bpy.context.scene.world.node_tree.nodes
# links = bpy.context.scene.world.node_tree.links

# c = colorsys.hsv_to_rgb(
#     random.uniform(0,255)/255, 
#     random.uniform(200,255)/255, 
#     random.uniform(200,255)/255
#     )
# # c = [c[0],c[1],c[2],1]
# c = [0,0,0,1]
# if len(nodes.get("Background").inputs['Color'].links) > 0:
#     links.remove(nodes.get("Background").inputs['Color'].links[0])

# nodes.get("Background").inputs['Strength'].default_value = 1
# nodes.get("Background").inputs['Color'].default_value = c 



# bpy.context.scene.cycles.samples =1
# bpy.context.view_layer.cycles.use_denoising = False
# bpy.context.scene.render.use_motion_blur = False
# bpy.context.scene.render.image_settings.file_format="OPEN_EXR"
# bpy.context.scene.render.image_settings.compression=0
# bpy.context.scene.render.image_settings.color_mode="RGBA"
# bpy.context.scene.render.image_settings.color_depth="32"
# bpy.context.scene.render.image_settings.exr_codec="NONE"
# # bpy.context.scene.render.image_settings.use_zbuffer=True
# bpy.context.view_layer.use_pass_z=True


# bpy.context.scene.use_nodes = True
# tree = bpy.context.scene.node_tree
# links = tree.links

# for n in tree.nodes:
#     tree.nodes.remove(n)

# # Create input render layer node.
# render_layers = tree.nodes.new('CompositorNodeRLayers')

# depth_file_output = tree.nodes.new(type="CompositorNodeOutputFile")
# depth_file_output.label = 'Depth Output'
# links.new(render_layers.outputs['Depth'], depth_file_output.inputs[0])
# depth_file_output.format.file_format = "OPEN_EXR"
# depth_file_output.base_path = f'{path}'

# node_viewer = tree.nodes.new('CompositorNodeViewer') 
# node_viewer.use_alpha = False  
# links.new(render_layers.outputs['Image'], node_viewer.inputs[0])


bpy.context.window.scene = bpy.data.scenes['Scene']

########################
########################
########################
########################
########################
########################



bpy.context.scene.render.filepath = config_file.output_path
bpy.ops.wm.save_as_mainfile(filepath=f"{config_file.output_path}/urdf.blend")


raise()


for i in range(NB_FRAMES):
    to_add = render_single_image(
        frame_set = i, 
        path = config_file.output_path,
        save_segmentation = True,  
        save_depth = True, 
        resolution = 1024, 
    )




########################
########################
########################
########################
########################
########################




