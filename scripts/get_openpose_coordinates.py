import argparse
import os
import json
import numpy as np
import matplotlib.pyplot as plt
# import openpose as op

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--json_dir', default=max,
                    help='directory of json files')
  args = parser.parse_args()
  listdir = os.listdir(args.json_dir)
  listdir.sort()
  print(listdir)

  body_parts = ["Nose",
                "Neck", 
                "RShoulder",
                "RElbow",
                "RWrist", 
                "LShoulder",
                "LElbow",
                "LWrist", 
                "MidHip",
                "RHip",
                "RKnee", 
                "RAnkle",
                "LHip",
                "LAnkle", 
                "REye",
                "LEye",
                "REar", 
                "LEar",
                "LBigToe", 
                "LSmallToe",
                "LHeel", 
                "RBigToe",
                "RSmallToe", 
                "RHeel",
                "Background"]

  body_dictionary = {"Nose": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "Neck": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "RShoulder": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "RElbow": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "RWrist": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "LShoulder": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LElbow": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LWrist": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "MidHip": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "RHip": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "RKnee": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "RAnkle": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LHip": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LAnkle": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "REye": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LEye": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "REar": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "LEar": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LBigToe": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "LSmallToe": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "LHeel": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "RBigToe": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "RSmallToe": [np.zeros([]), np.zeros([]), np.zeros([])], 
                   "RHeel": [np.zeros([]), np.zeros([]), np.zeros([])],
                   "Background": [np.zeros([]), np.zeros([]), np.zeros([])]}

  for json_file in listdir:
    loaded_json = json.load(open(args.json_dir+'/'+json_file))
    all_points = loaded_json['people'][0]['pose_keypoints_2d']
    # Structure of the json data points is: X, Y, Confidence, X2, Y2, Confidence2, ...
    for i in range(len(all_points)):
      body_index = int(i/3)
      body_dictionary[body_parts[body_index]][i % 3] = np.append(body_dictionary[body_parts[body_index]][i % 3], all_points[i])

  # Delete the zero elements
  for element in body_dictionary:
    for i in range(3):
      body_dictionary[element][i] = np.delete(body_dictionary[element][i], 0)

  plt.plot(body_dictionary["Nose"][0], body_dictionary["Nose"][1])
  plt.show()

if __name__ == '__main__':
    main()