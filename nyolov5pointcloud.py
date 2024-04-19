import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from torchvision.transforms import functional as F
from PIL import Image
import subprocess

class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-34), math.radians(0)
        self.translation = np.array([0, 0, 0], dtype=np.float32)
        self.distance = 0.6
        # self.pitch, self.yaw = math.radians(0), math.radians(0)
        # self.translation = np.array([0, 0, 0], dtype=np.float32)
        # self.distance = 0
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True
        
    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 0
        self.translation[:] = 0, 0, 0

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

state = AppState()

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 15)
config.enable_stream(rs.stream.color, rs.format.bgr8, 15)

pipeline.start(config)

profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()

cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(state.WIN_NAME, 1280, 720) 

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='brobo1.pt') #--------------------------------------------------------------------------------------------------------------------------------------------------------
model.eval() 

def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # Scale the projected points
    scale_factor = 2  # You can adjust this factor as needed
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect*scale_factor, h*scale_factor) + (w/2.0*scale_factor, h/2.0*scale_factor)

    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj

def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def pointcloud(out, verts, texcoords, depth_color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    np.clip(texcoords, 0, 1, out=texcoords)  # clip texcoords to [0, 1]

    cw, ch = depth_color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = depth_color[u[m], u[m]]



def process_frame(frame):
    # Convert frame to PIL image
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))


    # Perform object detection on the frame
    results = model(pil_image)

    # Get the detected objects' information
    detections = results.pred[0]

    # Draw vertical line in the middle of the image
    cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (255, 0, 0), 2)
    
    horizontal_line_y = frame.shape[0] // 2  # Calculate the y-coordinate for the horizontal line
    cv2.line(frame, (0, horizontal_line_y), (frame.shape[1], horizontal_line_y), (0, 0, 255), 2)

    
    
    

    # Iterate through detections
    for detection in detections:
        bbox = detection[:4].cpu().numpy()  # Get bounding box coordinates
        class_id = int(detection[5])        # Get predicted class id
        confidence = detection[4].item()    # Get confidence score

        if confidence > 0.5:  # Adjust this threshold as needed
            x, y, w, h = bbox
            x, y, w, h = int(x), int(y), int(w), int(h)

            # Reduce box size
            reduction_factor = 0.1  # Adjust as needed
            w = int(w * reduction_factor)
            h = int(h * reduction_factor)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Get class label and accuracy from the model
            class_label = model.names[class_id]
            accuracy = confidence * 100

            # Put class label and accuracy on the image
            label = f'{class_label}, {accuracy:.2f}%'
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Check if middle of the box aligns with the vertical line
            box_middle_x = x + w // 2
            box_middle_y = (y + h // 2) - 70 - 97.3 # 97.3 px of gripper distance, 70 px of detection box from conveyor
            middle_line_x = frame.shape[1] // 2
            middle_line_y = frame.shape[0] // 2
            
            pixcm = 9.73 # Pixel to CM value
            
            if abs(box_middle_x - middle_line_x) < 2:  # Adjust threshold as needed
                distance_y = ((middle_line_y - box_middle_y)/pixcm)
                alignment_text = f"X Aligned, Y {distance_y:.2f}cm"
                print(f"Allign, Y {distance_y:.2f}cm")
                command = "ls"
                # try:
                #     # Run the command and capture its output and error
                #     result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    
                #     # If the command ran successfully (exit code 0), print the output
                #     if result.returncode == 0:
                #         print("Command output:")
                #         print(result.stdout)
                #     else:
                #         # If the command failed (non-zero exit code), print the error
                #         print("Command failed with error:")
                #         print(result.stderr)
                # except subprocess.CalledProcessError as e:
                #     # Handle errors raised by subprocess.run() if the command fails
                #     print(f"Command execution failed: {e}")
                # except Exception as e:
                #     # Handle other exceptions
                #     print(f"An error occurred: {e}")
                
            elif box_middle_x > middle_line_x:
                distance = ((middle_line_x - box_middle_x)/pixcm)*-1
                distance_y = ((middle_line_y - box_middle_y)/pixcm)
                alignment_text = f"move right ({distance: .2f}cm, {distance_y:.2f}cm)"
                print(f"Move Right ({distance: .2f}px from the middle line)")
                command = "ls"
                # try:
                #     # Run the command and capture its output and error
                #     result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    
                #     # If the command ran successfully (exit code 0), print the output
                #     if result.returncode == 0:
                #         print("Command output:")
                #         print(result.stdout)
                #     else:
                #         # If the command failed (non-zero exit code), print the error
                #         print("Command failed with error:")
                #         print(result.stderr)
                # except subprocess.CalledProcessError as e:
                #     # Handle errors raised by subprocess.run() if the command fails
                #     print(f"Command execution failed: {e}")
                # except Exception as e:
                #     # Handle other exceptions
                #     print(f"An error occurred: {e}")
                
            elif box_middle_x < middle_line_x:
                distance = ((box_middle_x - middle_line_x)/pixcm)
                distance_y = ((middle_line_y - box_middle_y)/pixcm)
                alignment_text = f"move left ({distance: .2f}cm, {distance_y:.2f}cm)"
                print(f"Move Left ({distance: .2f}cm from the middle line)")
                command = "ls"
                # try:
                #     # Run the command and capture its output and error
                #     result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    
                #     # If the command ran successfully (exit code 0), print the output
                #     if result.returncode == 0:
                #         print("Command output:")
                #         print(result.stdout)
                #     else:
                #         # If the command failed (non-zero exit code), print the error
                #         print("Command failed with error:")
                #         print(result.stderr)
                # except subprocess.CalledProcessError as e:
                #     # Handle errors raised by subprocess.run() if the command fails
                #     print(f"Command execution failed: {e}")
                # except Exception as e:
                #     # Handle other exceptions
                #     print(f"An error occurred: {e}")
                
            
            else:
                print("Not Detected")

            # Put alignment information on the image
            cv2.putText(frame, alignment_text, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return frame

while True:
    if not state.paused:
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = decimate.process(depth_frame)

        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())

        if state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap

        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    out = np.empty((h, w, 3), dtype=np.uint8)

    now = time.time()

    out.fill(50)

    if not state.scale or out.shape[:2] == (h, w):
        pointcloud(out, verts, texcoords, depth_colormap)
    else:
        tmp = np.zeros((h, w, 3), dtype=np.uint8)
        pointcloud(tmp, verts, texcoords, depth_colormap)
        tmp = cv2.resize(
            tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        np.putmask(out, tmp > 0, tmp)

    dt = time.time() - now

    cv2.setWindowTitle(
        state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
        (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

    out = process_frame(out)  # Process frame with YOLOv5 object detection

    cv2.imshow(state.WIN_NAME, out)
    key = cv2.waitKey(1)
