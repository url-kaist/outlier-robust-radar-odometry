import numpy as np
import argparse
parser = argparse.ArgumentParser(description="Check Mean Errors")
parser.add_argument('--file_path', '-f', default="", type=str, help="Absolute path of the result file")
args = parser.parse_args()

data = np.loadtxt(args.file_path, delimiter=",")
print(data.shape)
is_not_stopped = (data[:, 1] == 0)
filtered = data[is_not_stopped, :]
print(filtered.shape)

# Unit: microseconds
time_total = data[:, 0] / 1000.0;
time_extracted = filtered[:, 0] / 1000.0;

print("Mean total time: " + str(np.mean(time_total)) + " ms")
print("Mean when it moves: " + str(np.mean(time_extracted)) + " ms")


