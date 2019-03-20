import numpy as np

def line(mat1, mat2):
    d = (mat2[1] - mat1[1]) / (mat2[0] - mat1[0])
    x = d
    y = -1
    c = d * mat1[0] - mat1[1]
    return x, y, c

def getIntersection(c1, c2, c3, c4):
    fx1, fy1, s1 = line(c1, c3)
    fx2, fy2, s2 = line(c2, c4)

    coef = np.array([[fx1, fy1], [fx2, fy2]])
    dep = np.array([c1, c2])

    return np.linalg.solve(coef, dep)
