import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

df = pd.read_csv('data/roll_only_Skeleton.csv')
# print(df['Neck_PositionX'])

time_df = pd.DataFrame({'timestamp': df['Time']/(10000000)})
quat_df = pd.DataFrame({'w':df['Neck_RotationW'], 'x':df['Neck_RotationX'], 'Y':df['Neck_RotationY'], 'z':df['Neck_RotationZ']})
rot = Rotation.from_quat(quat_df).as_euler('xyz', degrees=True)
euler_df = pd.DataFrame(data=rot, columns=['x', 'y', 'z'])
print(type(euler_df['x']))

result = pd.concat([time_df, euler_df['x']], axis=1, sort=False)
result.to_csv('roll.csv', index=False)

plt.subplot(131)
plt.plot(euler_df['x'])

plt.subplot(132)
plt.plot(euler_df['y'])

plt.subplot(133)
plt.plot(euler_df['z'])

plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(df['Neck_PositionX'], df['Neck_PositionY'], df['Neck_PositionZ'], c='r', marker='o')
# plt.show()