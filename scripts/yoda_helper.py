import numpy as np


def csv_to_servo_angles(csv_file, servo_angles):
  yoda_kinect_data_csv = np.genfromtxt(csv_file, delimiter=',')
  # Read from roll file.
  shoulder_roll_data_csv = yoda_kinect_data_csv[:,1]
  shoulder_pitch_data_csv = yoda_kinect_data_csv[:,2]
  elbow_data_csv = yoda_kinect_data_csv[:,3]
  pose_servo_record = []
  for i in range(yoda_kinect_data_csv.shape[0]):
      # Shoulder 1
      servo_angles[3] = shoulder_roll_data_csv[i]
      # Shoulder 2
      servo_angles[4] = shoulder_pitch_data_csv[i]
      # Elbow
      servo_angles[5] = np.clip(-elbow_data_csv[i], 0, 180)
      pose_servo_record.append([servo_angles.copy(), yoda_kinect_data_csv[i,0]])
  return pose_servo_record