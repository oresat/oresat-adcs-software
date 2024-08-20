import numpy as np
from scipy import optimize

from . import vector





def triad_algorithm(r1, r2, b1, b2):
    '''Essentially the TRIAD algorithm, and technically suboptimal. For details refer to
    "Fast Quaternion Attitude Estimation from Two Vector Measurements" Markely 2002.

    Parameters
        r1 : numpy.ndarray : More accurate inertial estimate.
        r2 : numpy.ndarray : Less accurate inertial estimate.
        b1 : numpy.ndarray : More accurate body measurement.
        b2: numpy.ndarray : Less accurate body measurement.

    Returns
        numpy.ndarray or bool : Quaternion attitude or False if solver failed.
    '''
    # still not handling a couple degenerate cases very well...
    r3       = np.cross(r1, r2)
    b3       = np.cross(b1, b2)
    if r3 is np.zeros(3) or b3 is np.zeros(3):
        return False
    if b1 is -r1:
        return False

    dot_term = 1 + np.dot(b1, r1)
    mu       = dot_term * np.dot(b3, r3) - np.dot(b1, r3) * np.dot(r1, b3)
    sum_term = b1 + r1
    nu       = np.dot(sum_term, np.cross(b3, r3))
    rho      = np.sqrt(mu**2 + nu**2)
    if mu >= 0:
        rhoplusmu = rho + mu
        rhomudot  = rhoplusmu * dot_term
        mult   = 0.5 * 1 / np.sqrt(rho * rhomudot)
        v      = rhoplusmu * np.cross(b1, r1) + nu * sum_term
        q_part = np.array([rhomudot, *v])
        q      = mult * q_part
    else:
        rhominmu = rho - mu
        mult   = 0.5 * 1 / np.sqrt(rho * rhominmu * dot_term)
        v      = nu * np.cross(b1, r1) + rhominmu * sum_term
        q_part = np.array([nu * dot_term, *v])
        q      = mult * q_part
    return q

def flae(a, r, b):
    # could precompute constants
    # not actually any faster, i've been lied to
    def sum_across(a, r, b, n, i, j):
        return sum([a[k] * r[k][i] * b[k][j] for k in range(n)])

    def f(x, t1, t2, t3):
        return x**4 + t1 * x**2 + t2 * x + t3

    def f_p(x, t1, t2, t3):
        return 4 * x**3 + 2 * t1 * x + t2

    r_hat  = [vector.normalize(ri) for ri in r]
    b_hat  = [vector.normalize(bi) for bi in b]
    n      = len(a)

    Hx = np.array([sum_across(a, r_hat, b_hat, n, 0, j) for j in range(3)])
    Hy = np.array([sum_across(a, r_hat, b_hat, n, 1, j) for j in range(3)])
    Hz = np.array([sum_across(a, r_hat, b_hat, n, 2, j) for j in range(3)])

    W  = np.array([[Hx[0] + Hy[1] + Hz[2], -Hy[2] + Hz[1],        -Hz[0] + Hx[2],        -Hx[1] + Hy[0]],
                   [-Hy[2] + Hz[1],         Hx[0] - Hy[1] - Hz[2], Hx[1] + Hy[0],         Hx[2] + Hz[0]],
                   [-Hz[0] + Hx[2],         Hx[1] + Hy[0],         Hy[1] - Hx[0] - Hz[2], Hy[2] + Hz[1]],
                   [-Hx[1] + Hy[0],         Hx[2] + Hz[0],         Hy[2] + Hz[1],         Hz[2] - Hy[1] - Hx[0]]])

    tau1 = -2 * (np.dot(Hx, Hx) + np.dot(Hy, Hy) + np.dot(Hz, Hz))
    tau2 = 8 * np.dot(Hx, np.cross(Hz, Hy))
    #tau2 = 8 * (Hx[2]*Hy[1]*Hz[0] - Hx[1]*Hy[2]*Hz[0] - Hx[2]*Hy[0]*Hz[1] + Hx[0]*Hy[2]*Hz[1] + Hx[1]*Hy[0]*Hz[2] - Hx[0]*Hy[1]*Hz[2])
    tau3 = np.linalg.det(W)

    lam = optimize.newton(f, 1, fprime=f_p, args=(tau1, tau2, tau3), maxiter=4, rtol=0.00001)

    # tried and failed to get analytic method working
    '''tau1_sq = tau1**2
    tau1and3 = tau1_sq + 12 * tau3
    T0 = 2 * tau1**3 + tau2 * 27 * tau2 - 72 * tau1 * tau3
    print(T0**2, 4 * tau1and3**3)
    T1 = (T0 + np.sqrt(T0**2 - 4 * tau1and3**3))**0.3333333
    T2 = np.sqrt(2**0.666666 * T1 - 4 * tau1 + 2**1.3333333 * tau1and3 / T1)

    mult = 1 / (2 * np.sqrt(6))
    term1 = -T2**2
    term2 = -12 * tau1
    term3 = 12 * np.sqrt(6) * tau2 / T2

    lam = []
    lam.append(mult * (T2 - np.sqrt(term1 + term2 - term3)))
    lam.append(mult * (T2 + np.sqrt(term1 + term2 - term3)))
    lam.append(-mult * (T2 + np.sqrt(term1 + term2 + term3)))
    lam.append(-mult * (T2 - np.sqrt(term1 + term2 + term3)))

    dist = [abs(l - 1) for l in lam]
    i = np.argmin(dist)
    N = W - np.eye(4) * lam[i]'''

    N = W - np.eye(4) * lam

    pivot = N[0][0]
    N[0] /= pivot
    N[1] -= N[1][0]*N[0]
    N[2] -= N[2][0]*N[0]
    N[3] -= N[3][0]*N[0]

    pivot = N[1][1]
    N[1] /= pivot
    N[0] -= N[0][1]*N[1]
    N[2] -= N[2][1]*N[1]
    N[3] -= N[3][1]*N[1]

    pivot = N[2][2]
    N[2] /= pivot
    N[0] -= N[0][2]*N[2]
    N[1] -= N[1][2]*N[2]
    N[3] -= N[3][2]*N[2]

    v = np.array([N[j][3] for j in range(3)])
    return vector.normalize(np.array([*v, -1]))
