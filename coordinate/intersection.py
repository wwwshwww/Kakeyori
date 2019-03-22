import numpy as np

def line(mat1, mat2):
    d = (mat2[1] - mat1[1]) / (mat2[0] - mat1[0])
    x = d
    y = -1
    c = d * mat1[0] - mat1[1]
    return x, y, c

def getQuadIntersection(pos):
    c1 = pos[0]
    c2 = pos[1]
    c3 = pos[2]
    c4 = pos[3]
    fx1, fy1, s1 = line(c1, c3)
    fx2, fy2, s2 = line(c2, c4)

    coef = np.array([[fx1, fy1], [fx2, fy2]])
    dep = np.array([s1, s2])

    ans = np.linalg.solve(coef, dep)
    return (int(ans[0]), int(ans[1]))
