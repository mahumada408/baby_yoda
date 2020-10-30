import argparse
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

df = pd.read_csv('data/waves_worldpos.csv')

body_parts = ["Hips",
              "Spine", 
              "LeftShoulder",
              "LeftArm",
              "LeftForeArm", 
              "LeftHand",
              "LeftHandEnd",
              "RightShoulder",
              "RightArm",
              "RightForeArm", 
              "RightHand",
              "RightHandEnd",
              "Neck",
              "Head",
              "HeadEnd",
              "RightUpLeg",
              "RightLeg",
              "RightFoot",
              "RightToeBase",
              "RightToeBaseEnd",
              "LeftUpLeg",
              "LeftLeg",
              "LeftFoot",
              "LeftToeBase",
              "LeftToeBaseEnd",]
global previous_pitch
previous_pitch = 0.0

def get_3_point(df, body_name, index):
  """
  Gets the x, y, z values from a given body_name.
  Args:
      df (pandas dataframe): dataframe of data
      body_name (string): name of body part (e.g. 'Skel107:Neck).
      index (int): Current index.
  Returns:
      Tuple of x, y, z
  """
  x = df[body_name + '.X'][index]
  y = df[body_name + '.Y'][index]
  z = df[body_name + '.Z'][index]
  return x, y, z

def update_graph(iteration):
  """
  Update the data held by the scatter plot and therefore animates it.
  Args:
      iteration (int): Current iteration of the animation
  Returns:
      list: List of scatters (One per element) with new coordinates
  """
  # Rotation matrix
  theta = np.radians(0)
  c, s = np.cos(theta), np.sin(theta)
  Rx = np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
  Rz = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))

  x_list = []
  y_list = []
  z_list = []

  prefix = 'Skel49:'

  # Get unit vector
  x_neck, y_neck, z_neck = get_3_point(df, prefix + 'Neck', iteration)
  x_shoulder, y_shoulder, z_shoulder = get_3_point(df, prefix + 'RightShoulder', iteration)
  r_NS = np.array([x_shoulder - x_neck, y_shoulder - y_neck, z_shoulder - z_neck])
  dir_NS = r_NS/np.linalg.norm(r_NS)

  x_right_arm, y_right_arm, z_right_arm = get_3_point(df, prefix + 'RightForeArm', iteration)
  r_S_RA = np.array([x_right_arm - x_shoulder, y_right_arm - y_shoulder, z_right_arm - z_shoulder])
  norm_r_S_RA = r_S_RA/np.linalg.norm(r_S_RA)
  shoulder_roll = np.arctan2(r_S_RA[1], r_S_RA[0]) * 180/np.pi
  shoulder_pitch = np.arctan2(r_S_RA[2], r_S_RA[1]) * 180/np.pi
  print(f"roll: {shoulder_roll}, pitch: {shoulder_pitch}")
  for part in body_parts:
    x, y, z = get_3_point(df, prefix+part, iteration)
    new_new = np.dot(Rz,np.array([x,y,z]))
    x_list.append(x)
    y_list.append(y)
    z_list.append(z)
  sct.set_data(x_list, y_list)
  sct.set_3d_properties(z_list)

def angle_between(vector_1, vector_2):
  return np.arccos(np.dot(vector_1, vector_2)/(np.linalg.norm(vector_1) * np.linalg.norm(vector_2))) * 180/np.pi

parser = argparse.ArgumentParser()
parser.add_argument('--show_plot', action='store_true',
                  help='Show visual animation')
args = parser.parse_args()

prefix = 'Skel49:'

if args.show_plot:
  # Attaching axis to the figure
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.set(xlabel='x-label', ylabel='y-label', zlabel='z')

  # Initialize scatter plot
  sct, = ax.plot([], [], [], "o", markersize=2)

  # Number of iterations
  iterations = len(df[prefix+'RightShoulder.X'])

  interval_ms = 1000 * (df['Time'][1] - df['Time'][0])
  ani = FuncAnimation(fig, update_graph, iterations, interval=interval_ms, blit=False, repeat=True)

  plt.show()

shoulder_pitch_list = []
shoulder_roll_list = []
elbow_angle_list = []
for i in range(len(df[prefix + 'RightShoulder.X'])):

  x_shoulder, y_shoulder, z_shoulder = get_3_point(df, prefix + 'RightShoulder', i)
  x_right_arm, y_right_arm, z_right_arm = get_3_point(df, prefix + 'RightForeArm', i)
  x_elbow, y_elbow, z_elbow = get_3_point(df, prefix + 'RightHand', i)

  r_S_RA = np.array([x_right_arm - x_shoulder, y_right_arm - y_shoulder, z_right_arm - z_shoulder])
  r_RA_E = np.array([x_elbow - x_right_arm, y_elbow - y_right_arm, z_elbow - z_right_arm])

  shoulder_roll = np.arctan2(r_S_RA[1], r_S_RA[0]) * 180/np.pi
  shoulder_pitch = np.arctan2(r_S_RA[2], r_S_RA[1]) * 180/np.pi
  elbow_angle = angle_between(r_S_RA, r_RA_E)
  
  shoulder_pitch_list.append(shoulder_pitch)
  shoulder_roll_list.append(shoulder_roll)
  elbow_angle_list.append(elbow_angle)
  print(f"roll: {shoulder_roll}, pitch: {shoulder_pitch}, elbow: {elbow_angle}")

# plt.scatter(df['Time'],elbow_angle_list)
# plt.show()

desired_frequency_hz = 100
total_points = int((df['Time'].iat[-1] - df['Time'].iat[0]) * desired_frequency_hz)
hz_100_timeline = np.linspace(df['Time'].iat[0],df['Time'].iat[-1], num=total_points).reshape(total_points, 1)

shoulder_pitch_100hz = np.interp(hz_100_timeline, np.array(df['Time'].tolist()), np.array(shoulder_pitch_list)).reshape(total_points,1)
shoulder_roll_100hz = np.interp(hz_100_timeline, np.array(df['Time'].tolist()), np.array(shoulder_roll_list)).reshape(total_points,1)
elbows_100hz = np.interp(hz_100_timeline, np.array(df['Time'].tolist()), np.array(elbow_angle_list)).reshape(total_points,1)

print(hz_100_timeline.shape)
print(elbows_100hz.shape)
arm_angles_100hz = np.concatenate((hz_100_timeline, shoulder_roll_100hz, shoulder_pitch_100hz, elbows_100hz), axis=1).reshape(total_points, 4)
np.savetxt("arm_angles_100.csv", arm_angles_100hz, delimiter=',')

plt.subplot(311)
plt.scatter(arm_angles_100hz[:,0], arm_angles_100hz[:,1])

plt.subplot(312)
plt.scatter(arm_angles_100hz[:,0], arm_angles_100hz[:,2])

plt.subplot(313)
plt.scatter(arm_angles_100hz[:,0], arm_angles_100hz[:,3])

plt.show()
