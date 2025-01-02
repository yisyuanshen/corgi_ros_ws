import numpy as np
from math import pow
import matplotlib.pyplot as plt


class Bezier:
    def __init__(self, control_pts):
        self.control_pts = control_pts
        self.bz_cff = self.bz_coeff(control_pts)

    def fact(self, n):
        # factorial
        return 1 if (n == 0) or (n == 1) else n * self.fact(n - 1)

    def comb(self, n, k):
        # Combination
        return self.fact(n) // (self.fact(k) * self.fact(n - k))

    def bz_coeff(self, cp_list):
        sz = len(cp_list)
        bzc = []
        for i in range(sz):
            bzc.append(self.comb(sz - 1, i))
        return bzc

    def bzt_coeff(self, cp_list, t):
        sz = len(cp_list)
        bzc = []
        for i in range(sz):
            ord_t_1 = (sz - 1) - i
            ord_t = i
            bzc.append(pow((1 - t), ord_t_1) * pow(t, ord_t))
        return bzc

    def getBzPoint(self, t, offset_x=0, offset_y=0):
        bzt_cff = self.bzt_coeff(self.control_pts, t)
        x = 0
        y = 0
        for i in range(len(self.control_pts)):
            cp_x = self.control_pts[i][0]
            cp_y = self.control_pts[i][1]
            x += bzt_cff[i] * self.bz_cff[i] * cp_x
            y += bzt_cff[i] * self.bz_cff[i] * cp_y
        x += offset_x
        y += offset_y
        return np.array([x, y])


if __name__ == "__main__":
    # Define the control points
    control_points = [np.array([0, 0]), np.array([1, 2]), np.array([3, 3])]

    # Create a Bezier object
    bezier_curve = Bezier(control_points)

    # Generate the Bezier curve points
    t_values = np.linspace(0, 1, 100)
    curve_points = [bezier_curve.getBzPoint(t) for t in t_values]

    # Extract x and y coordinates of the curve
    curve_x = [point[0] for point in curve_points]
    curve_y = [point[1] for point in curve_points]

    # Extract x and y coordinates of the control points
    control_x = [point[0] for point in control_points]
    control_y = [point[1] for point in control_points]

    # Plot the Bezier curve
    plt.plot(curve_x, curve_y, label="Bezier Curve")

    # Plot the control points and control polygon
    plt.plot(control_x, control_y, "ro-", label="Control Points")

    # Add labels and a legend
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()

    # Show the plot
    plt.show()
