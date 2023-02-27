import numpy as np
import argparse
parser = argparse.ArgumentParser(description="Check Mean Errors")
parser.add_argument('--file_path', '-f', default="", type=str, help="Absolute path of the result file")
args = parser.parse_args()

data = np.loadtxt(args.file_path, delimiter=",")
mean_data = np.mean(data, axis=0)

print("Mean traslation error: " + str(mean_data[0]) + " m")
print("Mean rotation error: " + str(mean_data[1]) + " deg")
print("Mean traslation perc. error: " + str(mean_data[2]) + " %")
print("Mean rotation deg/m error: " + str(mean_data[3]) + " deg/m")
