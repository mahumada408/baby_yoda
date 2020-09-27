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

  body_dictionary = {"Nose": np.zeros((1,3)),
                   "Neck": np.zeros((1,3)), 
                   "RShoulder": np.zeros((1,3)),
                   "RElbow": np.zeros((1,3)),
                   "RWrist": np.zeros((1,3)), 
                   "LShoulder": np.zeros((1,3)),
                   "LElbow": np.zeros((1,3)),
                   "LWrist": np.zeros((1,3)), 
                   "MidHip": np.zeros((1,3)),
                   "RHip": np.zeros((1,3)),
                   "RKnee": np.zeros((1,3)), 
                   "RAnkle": np.zeros((1,3)),
                   "LHip": np.zeros((1,3)),
                   "LAnkle": np.zeros((1,3)), 
                   "REye": np.zeros((1,3)),
                   "LEye": np.zeros((1,3)),
                   "REar": np.zeros((1,3)), 
                   "LEar": np.zeros((1,3)),
                   "LBigToe": np.zeros((1,3)), 
                   "LSmallToe": np.zeros((1,3)),
                   "LHeel": np.zeros((1,3)), 
                   "RBigToe": np.zeros((1,3)),
                   "RSmallToe": np.zeros((1,3)), 
                   "RHeel": np.zeros((1,3)),
                   "Background": np.zeros((1,3))}

  for json_file in listdir:
    loaded_json = json.load(open(args.json_dir+'/'+json_file))
    all_points = loaded_json['people'][0]['pose_keypoints_2d']
    # Structure of the json data points is: X, Y, Confidence, X2, Y2, Confidence2, ...
    for i in range(len(all_points)):
      body_index = int(i/3)
      if (i % 3) == 0:
        # Make a new row
        body_dictionary[body_parts[body_index]] = np.append(body_dictionary[body_parts[body_index]], np.zeros([1,3]), axis=0)
      body_dictionary[body_parts[body_index]][-1,i % 3] = all_points[i]

  # Delete the zero elements
  for element in body_parts:
    body_dictionary[element] = np.delete(body_dictionary[element], 0, 0)  

  # plt.plot(body_dictionary["Nose"][0], body_dictionary["Nose"][1])
  # plt.show()

if __name__ == '__main__':
    main()