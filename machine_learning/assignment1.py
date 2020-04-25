import numpy as np
import matplotlib.pyplot as plt
import math
from numpy import *

meanvector = np.array([[2],[-1]])
covariancevector = np.array([[3, 0.75], [0.75, 1.5]])

def GenerateData():
    [eigenvalues, eigenvectors] = np.linalg.eig(covariancevector)
    l = np.matrix(np.diag(np.sqrt(eigenvalues)))
    Q = np.matrix(eigenvectors) * l

    empirical_mean = np.array([[0], [0]], float)
    empirical_cov = np.array([[0, 0], [0, 0]], float)

    data_list = []
    s = np.array([])
    for data in range(200):
        randomvector = (Q * np.random.randn(2, 1)) + meanvector
        plt.scatter([randomvector[0]],[randomvector[1]])
        data_list.append(randomvector)

        empirical_mean += randomvector

    empMean = empirical_mean/200

    for single_data in data_list:
        empirical_cov += np.matmul((single_data - empMean), (single_data - empMean).T)
    empCov = empirical_cov/200

    print("Empirical mean: ", empMean)
    print("empirical cov: ", empCov)
    plt.title('Sampling a 2D Gaussian variable')
    plt.show()

    #task b
    b_mean = 2 #from meanvector
    b_std  = np.sqrt(3) #from covariancevector [0][0] :)
    #s = np.concatenate(data_list, axis=1)
    s = np.random.normal(b_mean, b_std, 200)

    count, bins, ignored = plt.hist(s, 30, density=True)
    plt.plot(bins, 1/(b_std * np.sqrt(2 * np.pi)) * np.exp( - (bins - b_mean)**2 / (2 * b_std**2) ), linewidth=2, color='r')
    plt.title('PDF - p(x)')
    plt.show()


#TASK 2     
true_w0 = 1.26
true_w1 = 1.77
ALPHA   = 10
I       = np.identity(2)
LAMBDA  = ALPHA * 1
mu_w    = np.array([1.3, 1.7])

def wMAP(x, t):
    return np.matmul(np.linalg.inv(np.matmul(x.T, x) + (LAMBDA * I)) , ((LAMBDA * mu_w) + np.matmul(x.T, t) ) )

def wML_old(num_points, x, t):
    w0 = 0
    w1 = 0
    for i in range(num_points):
        w0 += (t[i] - true_w1*x[i])
    w0 = w0 / num_points

    for i in range(num_points):
        w1 += (t[i]*x[i] - true_w0*x[i])
    temp = 0
    for i in x:
        temp += i**2
    w1 = w1 / temp
    return w0, w1

def wML(num_points, x, t):
    #return np.matmul(np.linalg.inv(np.matmul(x.T, x)), np.matmul(x.T, t))
    retval = np.matmul(np.linalg.inv(np.matmul(x.T, x)), np.matmul(x.T, t))
    print(retval.shape)
    return retval

def posteriorMeanCov(x, t):
    mean = np.matmul(np.linalg.inv(np.matmul(x.T, x) + LAMBDA * I), np.matmul(x.T, t) + LAMBDA*mu_w)
    cov  = np.linalg.inv(np.matmul(x.T, x) + LAMBDA * I)
    return mean, cov

def LinearRegression(num_points):

    x = np.random.uniform(-1, 1, num_points)
    e = np.random.randn(num_points)
    t = true_w0 + x*true_w1 + e

    actual_x = np.ones((x.shape[0], 2))    
    actual_x[:, 0] = x   #w_MAP
    w_MAP = wMAP(actual_x, t)

    #w_ML
    # derivative of w and equalize with 0 (for both w0 and w1)
    #[1/2 E (w0 + w1*x - ti)**2]'
    w_ML = wML(num_points, actual_x, t); 
    
    plot_pos = 311
    if num_points == 20:
        plot_pos = 312
    if num_points == 100:
        plot_pos = 313
    
    plt.subplot(plot_pos)

    plt.scatter(x, t, c = 2 * np.pi * np.random.rand(num_points))
    lins = np.linspace(-1, 1, num=4)

    y = true_w0 + lins*true_w1
    plt.plot(lins, y, color='g', label='true line')

    y = w_MAP[0] + lins*w_MAP[1]
    plt.plot(lins, y, color='r', label='w_MAP')

    y = w_ML[0] + lins*w_ML[1]
    plt.plot(lins, y, color='b', label='w_ML')

    plt.xlabel('t_i')
    plt.ylabel('x_i')
    legend = plt.legend(loc='lower right', shadow=True, fontsize='x-large')

    plt.legend()

    #mean and covariance of posterior
    post_mean, post_cov = posteriorMeanCov(actual_x, t)
    print("For num_points = ", num_points)
    print("MAP Estimate ", w_MAP)
    print("ML Estimate ", w_ML)
    print("Posterior mean: ", post_mean, "Posterior cov: ", post_cov)

    return w_MAP, w_ML

if __name__ == '__main__':
    GenerateData()
    w_MAP = [0]*3
    w_ML = [0]*3
    n_data = [3, 20, 100]

    w_MAP[0], w_ML[0] = LinearRegression(n_data[0])
    w_MAP[1], w_ML[1] = LinearRegression(n_data[1])
    w_MAP[2], w_ML[2] = LinearRegression(n_data[2])

    # fig, axes = plt.subplots((3, 3), sharex=True, sharey=True, figsize=(10,10))
    # w_min = 0
    # w_max = 2

    # point_grid = np.meshgrid(np.linspace(w_min, w_max, 50), np.linspace(w_min, w_max, 50))
    # for i in range(3):
    #     cax = axes[0, 0]

    #     plt.imshow()
    plt.show()
