import matplotlib.pyplot as plt
import sys

rawArr = []
filteredArr = []
peaksArr = []
finalArr = []

f = open("data.txt")

for line in f.readlines():
	parts = line.split(",")

	raw = int(parts[0])
	filtered = int(parts[1])
	peak = int(parts[2])
	final = int(parts[3])

	rawArr.append(raw)
	filteredArr.append(filtered)
	peaksArr.append(peak)
	finalArr.append(final)

plt.plot(rawArr, 'g', filteredArr, 'r', peaksArr, 'b', finalArr, 'y')
plt.show()

