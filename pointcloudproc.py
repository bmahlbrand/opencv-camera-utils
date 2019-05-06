import pyrealsense2 as rs
import numpy as np
import argparse
import os.path
from open3d import *
import copy
import cv2


parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream resolution, fps and format to match the recorded.")
parser.add_argument("-i1", "--input1", type=str, help="Path to the bag file")
parser.add_argument("-i2", "--input2", type=str, help="Path to the bag file")
args = parser.parse_args()

timestamp1 = -1
timestamp2 = -1

if not args.input1:
	print("No input parameter has been given.")
	print("For help type --help")
	exit()

if os.path.splitext(args.input1)[1] != ".bag":
	print("The given file is not in the correct file format.")
	print("Only .bag files are accepted")
	exit()


def createFrameData(point_cloud, point_color, path):
	new_directory = path.replace('.bag','')
	new_directory_rgb = path.replace('.bag','') + "_rgb"
	try:
		os.mkdir(new_directory)
	except FileExistsError:
		print("Directory "+new_directory+" already exists")
	try:
		os.mkdir(new_directory_rgb)
	except FileExistsError:
		print("Directory "+new_directory+" already exists")
	
	frame_count = 1
	frame_count_rgb = 1
	for frame in point_cloud:
		np.savetxt(new_directory+"/"+"frame"+str(frame_count)+".out", np.asarray(frame))
		frame_count += 1
	for color in point_color:
		np.savetxt(new_directory_rgb+"/"+"frame"+str(frame_count_rgb)+".out", np.asarray(color))
		frame_count_rgb += 1


def createImageData(image, frame_no, path):
	new_directory = path.replace('.bag','') +"_imgs"
	try:
		os.mkdir(new_directory)
	except FileExistsError:
		print("Directory "+new_directory+" already exists")
	
	rgb_values = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
	cv2.imwrite(new_directory+"/"+"frame"+str(frame_no)+".png", rgb_values)
	#return rgb_values


def extractTimeStamp(timestamp1, timestamp2):
	if(timestamp1 < timestamp2):
		return timestamp2
	else:
		return timestamp1

def extractBagData(pipeline, config, path):
	try:
		#pipeline = rs.pipeline()
		#config = rs.config()
		#rs.config.enable_device_from_file(config, args.input)
		config.enable_stream(rs.stream.depth)
		config.enable_stream(rs.stream.color)

		profile = pipeline.start(config)
		depth_sensor = profile.get_device().first_depth_sensor()
		depth_scale = depth_sensor.get_depth_scale()

		align_to = rs.stream.color
		align = rs.align(align_to)

		frame_number = 1;

		point_cloud = []
		point_color = []

		#THIS IS HOW WE GET TIMESTAMPS!
		#print(aligned_depth_frame.timestamp)

		first_timestamp = -1
		timestamp_set = False
		while True:
			#print("#Start of "+ str(frame_number)+"#");
			point_cloud.append([]);
			point_color.append([]);

			frames = pipeline.wait_for_frames()
			aligned_frames = align.process(frames)
			aligned_depth_frame = aligned_frames.get_depth_frame()
			color_frame = aligned_frames.get_color_frame()
			if(aligned_depth_frame.timestamp == first_timestamp):
				break
			if(timestamp_set == False):
				first_timestamp = aligned_depth_frame.timestamp
				timestamp_set = True

			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())

			createImageData(color_image, frame_number, path)
			
			num_rows = depth_image.shape[0]
			num_cols = depth_image.shape[1]

			depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

			num_rows = depth_image.shape[0]
			num_cols = depth_image.shape[1]

			for r in range(0, num_rows, 5):
				for c in range(0, num_cols, 5):
					depth = aligned_depth_frame.get_distance(c, r)
					depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [c, r], depth)
					dist = np.linalg.norm( depth_point_in_meters_camera_coords - np.array((0,0,0)))
					if(depth_point_in_meters_camera_coords[0] != 0 and depth_point_in_meters_camera_coords[1] != 0 and depth_point_in_meters_camera_coords[2] != 0 and dist < 2):
						point_cloud[frame_number-1].append(depth_point_in_meters_camera_coords)
						point_color[frame_number-1].append(color_image[r][c]/255.0)

			#print("#End of "+ str(frame_number)+"#");
			frame_number += 1;

			

	finally:
		pipeline.stop()
		createFrameData(point_cloud, point_color, path)



pipeline_l = rs.pipeline()
config_l = rs.config()
rs.config.enable_device_from_file(config_l, args.input1)

pipeline_r = rs.pipeline()
config_r = rs.config()
rs.config.enable_device_from_file(config_r, args.input2)

extractBagData(pipeline_l, config_l, args.input1)
extractBagData(pipeline_r, config_r, args.input2)