from matplotlib import pyplot as plt


plt.xlim(0, 10)
plt.ylim(0, 10)
plt.grid()

f = open("env.txt", "r")
x_points = []
y_points = []

for line in f:
    point = line.split()
    x_points.append(point[0])
    y_points.append(point[1])
    print(x_points)
    print(y_points)


plt.scatter(x_points, y_points)
plt.show()
