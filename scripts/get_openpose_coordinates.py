import argparse
import os
import json
import numpy as np

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--json_dir', default=max,
                    help='directory of json files')
  args = parser.parse_args()
  listdir = os.listdir(args.json_dir)
  listdir.sort()
  print(listdir)

  for json_file in listdir:
    loaded_json = json.load(open(args.json_dir+'/'+json_file))
    print(loaded_json['people'][0]['pose_keypoints_2d'])
    all_points = loaded_json['people'][0]['pose_keypoints_2d']
    # Structure of the json data points is: X, Y, Confidence, X2, Y2, Confidence2, ...
    for point in all_points:
      
    exit(0)

if __name__ == '__main__':
    main()