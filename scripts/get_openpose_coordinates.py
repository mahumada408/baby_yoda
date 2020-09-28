import argparse
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.cm as cm
# import openpose as op

def animate_scatters(iteration, data, scatter):
    """
    Update the data held by the scatter plot and therefore animates it.
    Args:
        iteration (int): Current iteration of the animation
        data (list): List of the data positions at each iteration.
        scatters (list): List of all the scatters (One per element)
    Returns:
        list: List of scatters (One per element) with new coordinates
    """
    points = np.zeros((1,2))
    for part in data:
      points = np.append(points, np.zeros((1,2)), axis=0)
      points[-1, 0] = data[part][iteration, 0]
      points[-1, 1] = data[part][iteration, 1]
    points = np.delete(points, 0, 0)
    scatter.set_offsets(points)
    colors = cm.rainbow(np.linspace(0, 1, 25))
    scatter.set_color(colors)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--json_dir', default=max,
                    help='directory of json files')
  parser.add_argument('--show_plot', action='store_true',
                    help='Show visual animation')
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

  c, s = np.cos(np.pi), np.sin(np.pi)
  j = np.matrix([[c, s], [-s, c]])

  x_point = 0
  y_point = 0
  for json_file in listdir:
    loaded_json = json.load(open(args.json_dir+'/'+json_file))
    all_points = loaded_json['people'][0]['pose_keypoints_2d']
    # Structure of the json data points is: X, Y, Confidence, X2, Y2, Confidence2, ...
    for i in range(len(all_points)):
      body_index = int(i/3)
      if (i % 3) == 0:
        # Make a new row
        body_dictionary[body_parts[body_index]] = np.append(body_dictionary[body_parts[body_index]], np.zeros([1,3]), axis=0)
      if (i % 3) == 0:
        # Rotate x
        x_point = all_points[i]
      elif (i % 3) == 1:
        # Rotate y
        y_point = all_points[i]
      else:
        percent = all_points[i]
        point = np.dot(j, [x_point, y_point]).T
        body_dictionary[body_parts[body_index]][-1,0:3] = np.array([point[0], point[1], percent])

  # Delete the zero elements
  for element in body_parts:
    body_dictionary[element] = np.delete(body_dictionary[element], 0, 0)  

  print(body_dictionary["Nose"])

  if args.show_plot:
    # Attaching axis to the figure
    fig = plt.figure()
    ax = plt.axes(xlim=(-960, 0), ylim=(-720,0))

    # Initialize scatter
    scatter = ax.scatter(body_dictionary["Nose"][0,0], body_dictionary["Nose"][0,1])

    # Number of iterations
    iterations = len(body_dictionary["Nose"])

    ani = FuncAnimation(fig, animate_scatters, iterations, fargs=(body_dictionary, scatter),
                                        interval=40, blit=False, repeat=True)

    # anim.save('test.gif', writer='imagemagick')
    plt.show()
  
  face_angle_points = body_dictionary["LEar"][:,0:2] - body_dictionary["REar"][:,0:2]
  face_angles = np.arctan2(face_angle_points[:, 0], face_angle_points[:, 1]) * 180/np.pi
  print(face_angles[0] - face_angles)


if __name__ == '__main__':
    main()