import numpy as np
import cv2
import json

path = "renders/jaxon/"
frame_id = 0

# Load JSON data
json_path = f"{path}/{frame_id:03d}.json"
with open(json_path, "r") as file:
    data = json.load(file)

# Load image using OpenCV
img_path = f"{path}/{frame_id:03d}.png"
img = cv2.imread(img_path)

# Assuming the rest of the data manipulation remains the same
part_transform = [a['local_to_world_matrix'] for a in data['objects']]
part_transform = np.array(part_transform)

c2w = np.array(data['camera_data']['cam2world'])
c2w[:, 1] *= -1  # Inverting Y
c2w[:, 2] *= -1  # Inverting Z

# Camera intrinsics
intri = [data['camera_data']['intrinsics'][key] for key in ['fx', 'fy', 'cx', 'cy']]
intri = np.array([[intri[0], 0, intri[2]], [0, intri[1], intri[2]], [0, 0, 1]])

# Compute 3D joint positions
joint_3d = part_transform[:, :3, 3]
joint_cam = c2w[:3, :3].T @ (joint_3d[:, :, None] - c2w[:3, 3:])
joint_2d = intri @ joint_cam
joint_2d = joint_2d[:, :2, 0] / joint_2d[:, 2:, 0]

# Convert image to RGB for display
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Overlay points and text on the image
points = []
for obj in data['objects']:
    # points.append(obj['projected_cuboid'][-1])
    points.append(obj['projected_obj_origin'])
    
# for i, point in enumerate(joint_2d):

for i, point in enumerate(joint_2d):
    point = (int(point[0]), int(point[1]))
    cv2.circle(img_rgb, point, radius=5, color=(0, 0, 255), thickness=-1)
    # cv2.putText(img_rgb, str(i), point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


for i, point in enumerate(points):
    point = (int(point[0]), int(point[1]))
    # cv2.circle(img_rgb, point, radius=5, color=(0, 255, 0), thickness=-1)
    # cv2.putText(img_rgb, str(i), point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# Display the image
cv2.imshow('Image with Joints', img_rgb)
cv2.waitKey(0)  # Wait for a key press to close
cv2.destroyAllWindows()
