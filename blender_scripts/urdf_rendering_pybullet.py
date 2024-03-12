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

import pybullet
import pybullet as p
import pybullet_data

from icecream import ic 
print = ic 

parser = argparse.ArgumentParser(description='Renders given obj file by rotation a camera around it.')
parser.add_argument(
    '--config', default=None,
    help='skip the scene set up')

argv = sys.argv[sys.argv.index("--") + 1:]
opt = parser.parse_args(argv)

config_file = OmegaConf.load(opt.config)
base = OmegaConf.load(config_file.base)
config_file.base = base



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
bpy.context.scene.render.film_transparent = True

bpy.context.scene.cycles.device = 'GPU'
bpy.context.scene.cycles.diffuse_bounces = 1
bpy.context.scene.cycles.glossy_bounces = 1
bpy.context.scene.cycles.transparent_max_bounces = 3
bpy.context.scene.cycles.transmission_bounces = 3
bpy.context.scene.cycles.samples = 32
bpy.context.scene.cycles.use_denoising = True

bpy.context.scene.frame_end = config_file.render.nb_frames

##### LOAD THE URDF SCENE #####
# urdf_content_path = "/Users/jtremblay/code/yourdfpy/robot/kitchen_urdf/"
# kitchen = URDF.load(f"{urdf_content_path}/kitchen.urdf")

# # JACO
# urdf_content_path = config_file.content.urdf_content_path

# kitchen = URDF.load(f"{urdf_content_path}/{config_file.content.urdf_path}")

def get_world_link_pose(body_unique_id: int, link_index: int):
    """Pose of link frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    if link_index == -1:
        world_inertial_pose = get_world_inertial_pose(body_unique_id, -1)
        dynamics_info = p.getDynamicsInfo(body_unique_id, -1)
        local_inertial_pose = (dynamics_info[3], dynamics_info[4])

        local_inertial_pose_inv = p.invertTransform(local_inertial_pose[0], local_inertial_pose[1])
        pos_orn = p.multiplyTransforms(world_inertial_pose[0],
                                       world_inertial_pose[1],
                                       local_inertial_pose_inv[0],
                                       local_inertial_pose_inv[1])
    else:
        state = p.getLinkState(body_unique_id, link_index)
        pos_orn = (state[4], state[5])

    return pos_orn


def get_world_inertial_pose(body_unique_id: int, link_index: int):
    """Pose of inertial frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    if link_index == -1:
        pos_orn = p.getBasePositionAndOrientation(body_unique_id)
    else:
        state = p.getLinkState(body_unique_id, link_index)
        pos_orn = (state[0], state[1])

    return pos_orn


def get_world_visual_pose(body_unique_id: int, link_index: int):
    """Pose of visual frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    world_link_pose = get_world_link_pose(body_unique_id, link_index)
    visual_shape_data = p.getVisualShapeData(body_unique_id, link_index)
    local_visual_pose = (visual_shape_data[link_index + 1][5], visual_shape_data[link_index + 1][6])
    # local_visual_pose = (visual_shape_data[link_index][5], visual_shape_data[link_index][6])

    print(link_index,world_link_pose)

    return p.multiplyTransforms(world_link_pose[0],
                                world_link_pose[1],
                                local_visual_pose[0],
                                local_visual_pose[1])


def update_visual_objects(object_ids, pkg_path, nv_objects=None,i_frame=-1):
    
    # object ids are in pybullet engine
    # pkg_path is for loading the object geometries
    # nv_objects refers to the already entities loaded, otherwise it is going 
    # to load the geometries and create entities. 
    if nv_objects is None:
        nv_objects = { }
    for object_id in object_ids:
        for idx, visual in enumerate(p.getVisualShapeData(object_id)):
            # Extract visual data from pybullet
            objectUniqueId = visual[0]
            linkIndex = visual[1]
            visualGeometryType = visual[2]
            dimensions = visual[3]
            meshAssetFileName = visual[4]
            local_visual_frame_position = visual[5]
            local_visual_frame_orientation = visual[6]
            rgbaColor = visual[7]

            if linkIndex == -1:
                # dynamics_info = p.getDynamicsInfo(object_id,-1)
                # inertial_frame_position = dynamics_info[3]
                # inertial_frame_orientation = dynamics_info[4]
                # base_state = p.getBasePositionAndOrientation(objectUniqueId)
                # world_link_frame_position = base_state[0]
                # world_link_frame_orientation = base_state[1]    
                # m1 = nv.translate(nv.mat4(1), nv.vec3(inertial_frame_position[0], inertial_frame_position[1], inertial_frame_position[2]))
                # m1 = m1 * nv.mat4_cast(nv.quat(inertial_frame_orientation[3], inertial_frame_orientation[0], inertial_frame_orientation[1], inertial_frame_orientation[2]))
                # m2 = nv.translate(nv.mat4(1), nv.vec3(world_link_frame_position[0], world_link_frame_position[1], world_link_frame_position[2]))
                # m2 = m2 * nv.mat4_cast(nv.quat(world_link_frame_orientation[3], world_link_frame_orientation[0], world_link_frame_orientation[1], world_link_frame_orientation[2]))
                # m = nv.inverse(m1) * m2
                # q = nv.quat_cast(m)

                link_state = p.getLinkState(object_id,link_index)

                # print(link_state[0])
                # print(link_state[2])
                # print(link_state[4])

                # raise()

                dynamics_info = p.getDynamicsInfo(object_id, -1)
                inertial_frame_position = dynamics_info[3]
                inertial_frame_orientation = dynamics_info[4]
                base_state = p.getBasePositionAndOrientation(objectUniqueId)
                world_link_frame_position = base_state[0]
                world_link_frame_orientation = base_state[1]


                m1 = np.eye(4)
                m1[:3, 3] = inertial_frame_position
                m1[:3, :3] = pyrr.matrix44.create_from_quaternion(inertial_frame_orientation)[:3, :3]

                m2 = np.eye(4)
                m2[:3, 3] = world_link_frame_position
                m2[:3, :3] = pyrr.matrix44.create_from_quaternion(world_link_frame_orientation)[:3, :3]

                m3 = np.eye(4)
                m3[:3, 3] = link_state[4]
                m3[:3, :3] = pyrr.matrix44.create_from_quaternion(link_state[5])[:3, :3]



                m = np.linalg.inv(m1) @ m2 
                # q = pyrr.quaternion.matrix_to_quaternion(m)                
                q = pyrr.quaternion.create_from_matrix(m)
                world_link_frame_position = m[:3,3]
                world_link_frame_orientation = q

            else:
                linkState = p.getLinkState(objectUniqueId, linkIndex)
                world_link_frame_position = linkState[4]
                world_link_frame_orientation = linkState[5]
            
            # Name to use for components
            object_name = f"{objectUniqueId}_{linkIndex}_{idx}"

            meshAssetFileName = meshAssetFileName.decode('UTF-8')
            if object_name not in nv_objects:
                # Create mesh component if not yet made
                if visualGeometryType == p.GEOM_MESH:
                    # nv_objects[object_name] = nv.import_scene(
                    #     pkg_path + "/" + meshAssetFileName
                    # )

                    data_2_load = os.path.join(f'{pkg_path}',meshAssetFileName)
                    # print(data_2_load)
                    file_type = data_2_load.split('.')[-1]
                    # bpy.ops.import_scene.obj(filepath=data_2_load)
                    if file_type.lower() == 'obj':
                        bpy.ops.wm.obj_import(filepath=data_2_load)
                    elif file_type.lower() == 'dae':
                        bpy.ops.wm.collada_import(filepath=data_2_load)
                    elif file_type.lower() == 'stl':
                        bpy.ops.wm.stl_import(filepath=data_2_load)

                    delete_all_lights()
                    delete_all_cameras()

                    selected_objects = bpy.context.selected_objects

                    # Deselect all objects
                    bpy.ops.object.select_all(action='DESELECT')

                    # Select mesh objects among the selected objects
                    for obj in selected_objects:
                        if obj.type == 'MESH':
                            print(obj.name)
                            obj.select_set(True)
                    # Join selected mesh objects
                    if len(bpy.context.selected_objects) == 0:
                        continue
                    bpy.ops.wm.save_as_mainfile(filepath=f"{config_file.output_path}/urdf.blend")
                    # override = {'selected_objects': bpy.context.selected_objects}
                    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
                    bpy.ops.object.join()
                    # bpy.ops.object.join(override)

                    bpy.ops.object.shade_smooth(use_auto_smooth=True)
                            
                    obj = bpy.context.selected_objects[0]
                    obj.name = object_name
                    bpy.context.view_layer.objects.active = obj

                    nv_objects[object_name] = obj

                    add_annotation(obj,link_name= object_name)


                # elif visualGeometryType == p.GEOM_BOX:
                #     assert len(meshAssetFileName) == 0
                #     nv_objects[object_name] = nv.entity.create(
                #         name=object_name,
                #         mesh=nv.mesh.create_box(
                #             name=object_name,
                #             # half dim in NVISII v.s. pybullet
                #             size=nv.vec3(dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2)
                #         ),
                #         transform=nv.transform.create(object_name),
                #         material=nv.material.create(object_name),
                #     )
                # elif visualGeometryType == p.GEOM_CYLINDER:
                #     assert len(meshAssetFileName) == 0
                #     length = dimensions[0]
                #     radius = dimensions[1]
                #     nv_objects[object_name] = nv.entity.create(
                #         name=object_name,
                #         mesh=nv.mesh.create_cylinder(
                #             name=object_name,
                #             radius=radius,
                #             size=length / 2,    # size in nvisii is half of the length in pybullet
                #         ),
                #         transform=nv.transform.create(object_name),
                #         material=nv.material.create(object_name),
                #     )
                # elif visualGeometryType == p.GEOM_SPHERE:
                #     assert len(meshAssetFileName) == 0
                #     nv_objects[object_name] = nv.entity.create(
                #         name=object_name,
                #         mesh=nv.mesh.create_sphere(
                #             name=object_name,
                #             radius=dimensions[0],
                #         ),
                #         transform=nv.transform.create(object_name),
                #         material=nv.material.create(object_name),
                #     )
                else:
                    # other primitive shapes currently not supported
                    continue

            if object_name not in nv_objects: continue

            # # Link transform
            # m1 = nv.translate(nv.mat4(1), nv.vec3(world_link_frame_position[0], world_link_frame_position[1], world_link_frame_position[2]))
            # m1 = m1 * nv.mat4_cast(nv.quat(world_link_frame_orientation[3], world_link_frame_orientation[0], world_link_frame_orientation[1], world_link_frame_orientation[2]))

            # # Visual frame transform
            # m2 = nv.translate(nv.mat4(1), nv.vec3(local_visual_frame_position[0], local_visual_frame_position[1], local_visual_frame_position[2]))
            # m2 = m2 * nv.mat4_cast(nv.quat(local_visual_frame_orientation[3], local_visual_frame_orientation[0], local_visual_frame_orientation[1], local_visual_frame_orientation[2]))

            # nv_objects[object_name].get_transform().set_transform(m1 * m2)

            m1 = np.eye(4)
            m1[:3, 3] = world_link_frame_position
            m1[:3, :3] = pyrr.matrix44.create_from_quaternion(world_link_frame_orientation)[:3, :3]

            # Visual frame transform
            m2 = np.eye(4)

            m2[:3, 3] = local_visual_frame_position
            m2[:3, :3] = pyrr.matrix44.create_from_quaternion(local_visual_frame_orientation)[:3, :3]

            if "rot_x" in config_file:
                m = pyrr.Matrix44.from_x_rotation(config_file.rot_x) @ m1 @ m2 
            else:
                m = m1 @ m2
            # print(object_id,linkIndex)
            # local_visual_frame_position,local_visual_frame_orientation = \
            #     get_world_visual_pose(object_id,linkIndex)

            # m = np.eye(4)

            # m[:3, 3] = local_visual_frame_position
            # m[:3, :3] = pyrr.matrix44.create_from_quaternion(local_visual_frame_orientation)[:3, :3]

            # print(m)

            bpy.ops.object.select_all(action='DESELECT')
            nv_objects[object_name].select_set(True)

            nv_objects[object_name].location.x = m[0][-1]
            nv_objects[object_name].location.y = m[1][-1]
            nv_objects[object_name].location.z = m[2][-1]

            # print(trans)

            matrix = pyrr.Matrix44(m).matrix33
            # print(matrix)
            # matrix = matrix * pyrr.Matrix33.from_x_rotation(-np.pi/2)
            nv_objects[object_name].rotation_mode = 'QUATERNION'
            nv_objects[object_name].rotation_quaternion.x = matrix.quaternion.x
            nv_objects[object_name].rotation_quaternion.y = matrix.quaternion.y
            nv_objects[object_name].rotation_quaternion.z = matrix.quaternion.z
            nv_objects[object_name].rotation_quaternion.w = matrix.quaternion.w

            if i_frame > 0:     
                nv_objects[object_name].keyframe_insert(
                    data_path='location', 
                    frame=i_frame
                )
                nv_objects[object_name].keyframe_insert(
                    data_path='rotation_quaternion', 
                    frame=i_frame
                )
                # print('frame')
    return nv_objects


s = config_file.seed
random.seed(s)
np.random.seed(s)

NB_FRAMES = config_file.render.nb_frames
root_obj = None
all_textures = glob.glob(config_file.content.materials_path+"*/")


# p.connect(p.GUI)
p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(config_file.content.urdf_content_path)
robot = p.loadURDF(config_file.content.urdf_path, [0, 0, 0], useFixedBase=True)

num_joints = p.getNumJoints(robot)

# Store child link indices
child_links = set()

# Iterate over each joint and store child link indices
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot, joint_index)
    child_link_index = joint_info[16]
    if child_link_index != -1:
        child_links.add(child_link_index)

# Find the root link
root_link_index = None
for link_index in range(num_joints + 1):  # Including the base link
    if link_index not in child_links:
        root_link_index = link_index
        break



num_joints = p.getNumJoints(robot)
numJoints = p.getNumJoints(robot)

# Generate random joint poses
random_joint_poses = np.random.rand(num_joints) * 2.0 - 1.0  # Random values between -1 and 1

# random_joint_poses = np.zeros(num_joints)

# Set the joint poses
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot, joint_index)
    joint_type = joint_info[2]
    
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        # Set the joint pose
        p.resetJointState(robot, joint_index, random_joint_poses[joint_index])

limits = []
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot, joint_index)
    joint_name = joint_info[1].decode("utf-8")  # Decode the joint name
    joint_type = joint_info[2]  # Joint type (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, etc.)
    joint_lower_limit = joint_info[8]  # Lower limit of the joint
    joint_upper_limit = joint_info[9]  # Upper limit of the joint
    limits.append([joint_lower_limit,joint_upper_limit])



# Keep track of the cube objects
nv_objects = update_visual_objects([robot], "")
nv_objects = update_visual_objects([robot], ".", nv_objects)


if config_file.random_materials: 
    for obj_name in nv_objects:
        obj = nv_objects[obj_name]
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj 

        bpy.ops.object.mode_set(mode='EDIT')
        # # Select the geometry
        bpy.ops.mesh.select_all(action='SELECT')
        # # Call the smart project operator
        bpy.ops.uv.smart_project()
        # # Toggle out of Edit Mode
        bpy.ops.object.mode_set(mode='OBJECT')

        adding_material = True
        while adding_material:
            try:
                path = all_textures[np.random.randint(0,len(all_textures)-1)]
                print(path)
                add_material(obj,path)
                adding_material = False
            except:
                pass



# raise()

# timeStep = 1.0/240  # Adjust the time step of the simulation. Smaller values can lead to more accurate simulations
# numSolverIterations = 150  # Increase the solver iterations for better stability and accuracy

# p.setTimeStep(timeStep)
# p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)

# Initialize target positions with current positions
joint_targets = [p.getJointState(robot, i)[0] for i in range(num_joints)]
nb_frames = config_file.render.nb_frames
step_per_frame = config_file.nb_bullet_steps

if config_file.test_scene: 
    np.seed = config_file.seed +10


for i_frame in range(nb_frames):
    # Randomly decide new targets for some joints
    for i in range(num_joints):
        if np.random.rand() < config_file.prob_change_pose:  # 10% chance to update target for each joint
            if i in config_file.ignore_list:
                joint_targets[i] = joint_targets[i]
            else:
                joint_targets[i] = np.random.uniform(limits[i][0], limits[i][1])
    
    # Move towards the new targets smoothly
    for _ in range(step_per_frame):
        for i in range(num_joints):
            current_position = p.getJointState(robot, i)[0]
            # Calculate a step towards the target position
            step_size = (joint_targets[i] - current_position) * config_file.step_size  # Adjust step size for smoother movement
            new_position = current_position + step_size
            # Apply the new position within limits
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=new_position)
        
        p.stepSimulation()
    nv_objects = update_visual_objects([robot], ".", nv_objects, i_frame = i_frame)










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

# Create a new empty object
bpy.ops.object.empty_add(location=config_file.offset)
track_camera = bpy.context.active_object
track_camera.name = 'target'


    

bpy.ops.object.camera_add(location=(0, 0, 0))  # Set the initial location of the camera
camera_object_moving = bpy.context.object
camera_object_moving.name = 'Camera'  # Rename the camera object to 'Camera'
bpy.context.scene.camera = camera_object_moving  # Set the scene's active camera to the newly added camera

track_to_constraint = camera_object_moving.constraints.new(type='TRACK_TO')
track_to_constraint.target = track_camera


radius = 1.0

# Define parameters for the line
theta = np.linspace(0, 6*np.pi, config_file.render.nb_frames)  # Parameter for the line, making a full loop around the sphere

# To ensure the line stays on the sphere, adjust phi instead of z directly
# Add a sinusoidal variation to phi over theta to create interesting patterns on the sphere
phi = np.pi / 2 + config_file.amplitude_camera * np.sin(4.25 * theta)  # Sinusoidal variation around the equator

# Calculate the coordinates of the line on the sphere using spherical coordinates
x_line = radius * np.sin(phi) * np.cos(theta)
y_line = radius * np.sin(phi) * np.sin(theta)
z_line = radius * np.cos(phi)


for i in range(config_file.render.nb_frames):
    camera_object_moving.location.x = x_line[i]*config_file.render.camera.distance_center + config_file.offset[0]
    camera_object_moving.location.y = y_line[i]*config_file.render.camera.distance_center + config_file.offset[1]
    camera_object_moving.location.z = z_line[i]*config_file.render.camera.distance_center + config_file.offset[2] 
    
    camera_object_moving.keyframe_insert(data_path='location', frame=i)


all_textures = glob.glob(config_file.content.materials_path+"*/")

# test camera
if config_file.test_scene:

    points_circle = []
    for i in range(config_file.nb_cameras):
        theta = 2 * np.pi * i / config_file.nb_cameras  # Angle for the current point
        x = np.cos(theta)   # X coordinate
        y = np.sin(theta)  # Y coordinate
        points_circle.append((x, y))


    all_cameras = []
    for i_camera in range(config_file.nb_cameras):
        # add a camera 

        bpy.ops.object.camera_add(location=(0, 0, 0))  
        camera_object = bpy.context.object
        camera_object.name = f'Camera_{str(i_camera)}'  # Rename the camera object to 'Camera'
        all_cameras.append(camera_object)
        bpy.context.scene.camera = camera_object  # Set the scene's active camera to the newly added camera

        track_to_constraint = camera_object.constraints.new(type='TRACK_TO')

        track_to_constraint.target = track_camera
        camera_object.location.x = points_circle[i_camera][0]*config_file.render.camera.distance_center + config_file.offset[0]
        camera_object.location.y = points_circle[i_camera][1]*config_file.render.camera.distance_center + config_file.offset[1]
        camera_object.location.z = 0 + config_file.offset[2] 


########################
########################
########################
########################
########################
########################

##### CREATE A new scene for segmentation rendering 

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


# TODO HERE. 


# Set the scene camera (ensure a camera is selected)
scene = bpy.context.scene
if scene.camera is None and bpy.context.selected_objects:
    # If no active camera, use selected camera
    for obj in bpy.context.selected_objects:
        if obj.type == 'CAMERA':
            scene.camera = obj
            break

# Change the viewport to look through the active camera
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        area.spaces.active.region_3d.view_perspective = 'CAMERA'
        break

bpy.context.scene.render.filepath = config_file.output_path
bpy.ops.wm.save_as_mainfile(filepath=f"{config_file.output_path}/urdf.blend")
# raise()


for i in range(NB_FRAMES):
    if i >= 600: 
        break
    bpy.context.scene.camera = camera_object_moving
    to_add = render_single_image(
        frame_set = i, 
        path = config_file.output_path,
        save_segmentation = True,  
        save_depth = True, 
        resolution = 1024, 
    )

    # raise()
    if config_file.test_scene and i % config_file.test_every_frame == 0: 
        for i_cam in range(config_file.nb_cameras):
            bpy.context.scene.camera = all_cameras[i_cam]  # Set the scene's active camera to the newly added camera

            to_add = render_single_image(
                frame_set = i, 
                path = f"{config_file.output_path}/test/",
                save_segmentation = True,  
                save_depth = True, 
                resolution = 1024, 
                suffix_naming = f"_{str(i_cam)}",

            )
        # export mesh 
        file_path = f"{config_file.output_path}/test/{str(i).zfill(3)}.obj" 

        # Set up export settings
        export_settings = {
            "filepath": file_path,
            "export_selected_objects": False,
            "export_animation": False,
            "apply_modifiers": True,
            "export_materials":False,
        }

        # Execute the export operation
        bpy.ops.wm.obj_export(**export_settings)

    # raise()


########################
########################
########################
########################
########################
########################




