import numpy as np
import matplotlib.pyplot as plt

grid = np.loadtxt("ocgm_660610823.csv", delimiter=",")

plt.imshow(
    grid,
    cmap="gray_r",
    origin="lower"
)

plt.colorbar(label="Occupancy Probability")
plt.title("Occupancy Grid Map")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")
plt.axis("equal")
plt.show()