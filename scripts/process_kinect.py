import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def plot_angles(df, prefix):
  plt.subplot(131)
  plt.plot(df[prefix + 'roll'])

  plt.subplot(132)
  plt.plot(df[prefix + 'yaw'])

  plt.subplot(133)
  plt.plot(df[prefix + 'pitch'])

  plt.show()


df = pd.read_csv('data/extreme_right_up_down_Skeleton.csv')
# print(df['Neck_PositionX'])

time_df = pd.DataFrame({'timestamp': df['Time']/(10000000)})
# quat_df = pd.DataFrame({'w':df['Neck_RotationW'], 'x':df['Neck_RotationX'], 'Y':df['Neck_RotationY'], 'z':df['Neck_RotationZ']})
# rot = Rotation.from_quat(quat_df).as_euler('xyz', degrees=True)
# euler_df = pd.DataFrame(data=rot, columns=['head_roll', 'head_yaw', 'head_pitch'])

right_shoulder_quat_df = pd.DataFrame({'x':df['right_shoulder_RotationX'], 'y':df['right_shoulder_RotationY'], 'z':df['right_shoulder_RotationZ'], 'w':df['right_shoulder_RotationW'], })
right_shoulder_rotation = Rotation.from_quat(right_shoulder_quat_df).as_euler('xyz', degrees=True)
right_shoulder_euler_df = pd.DataFrame(data=right_shoulder_rotation, columns=['right_shoulder_roll', 'right_shoulder_yaw', 'right_shoulder_pitch'])

# x -> roll
# z -> pitch

# result = pd.concat([time_df,euler_df['x'], euler_df['z']], axis=1, sort=False)
# result.to_csv('right_shoulder.csv', index=False)

plot_angles(right_shoulder_euler_df, 'right_shoulder_')

# plt.subplot(131)
# plt.plot(euler_df['head_roll'])

# plt.subplot(132)
# plt.plot(euler_df['head_yaw'])

# plt.subplot(133)
# plt.plot(euler_df['head_pitch'])

# plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(df['Neck_PositionX'], df['Neck_PositionY'], df['Neck_PositionZ'], c='r', marker='o')
# plt.show()