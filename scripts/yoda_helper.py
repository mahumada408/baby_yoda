import numpy as np


def csv_to_servo_angles(csv_file):
  yoda_kinect_data_csv = np.genfromtxt(csv_file, delimiter=',')
  # Read from roll file.
  shoulder_roll_data_csv = yoda_kinect_data_csv[:,1]
  shoulder_pitch_data_csv = yoda_kinect_data_csv[:,2]
  elbow_data_csv = yoda_kinect_data_csv[:,3]
  pose_servo_record = []
  current_servo_angle = servo_angles
  for i in range(yoda_kinect_data_csv.shape[0]):
      # Shoulder 1
      servo_angles[3] = shoulder_roll_data_csv[i]
      # Shoulder 2
      servo_angles[4] = shoulder_pitch_data_csv[i]
      # Elbow
      current_servo_angle[5] = elbow_csv[i]
      pose_servo_record.append([current_servo_angle.copy(), yoda_kinect_data_csv[i,0]])
  return pose_servo_record