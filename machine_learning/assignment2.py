
import numpy as np
import plot
from utils import mnist_reader
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from numpy import linalg as la
import sys

np.set_printoptions(threshold=sys.maxsize)

x_train_ = []
y_train_ = []

def show_images(images, cols = 1, titles = None):
    """Display a list of images in a single figure with matplotlib.
    
    Parameters
    ---------
    images: List of np.arrays compatible with plt.imshow.
    
    cols (Default = 1): Number of columns in figure (number of rows is 
                        set to np.ceil(n_images/float(cols))).
    
    titles: List of titles corresponding to each image. Must have
            the same length as titles.
    """
    assert((titles is None)or (len(images) == len(titles)))
    n_images = len(images)
    if titles is None: titles = ['Image (%d)' % i for i in range(1,n_images + 1)]
    fig = plt.figure()
    for n, (image, title) in enumerate(zip(images, titles)):
        a = fig.add_subplot(cols, np.ceil(n_images/float(cols)), n + 1)
        if image.ndim == 2:
            plt.gray()
        plt.imshow(image)
        #a.set_title(title)
    fig.set_size_inches(np.array(fig.get_size_inches()) * n_images)
    plt.show()

def import_data():
    X_train_mnist, y_train_mnist = mnist_reader.load_mnist('data/fashion', kind='train')
    X_test_mnist, y_test_mnist = mnist_reader.load_mnist('data/fashion', kind='t10k')

    for i in range(y_train_mnist.size):
        if y_train_mnist[i] < 5:
            y_train_.append(y_train_mnist[i])
            x_train_.append(X_train_mnist[i])
    return np.array(x_train_), np.array(y_train_)

def scale_and_center():
    new_x_train = np.array(x_train_)
    new_x_train = new_x_train / 255
    new_x_train = new_x_train - np.mean(new_x_train)
    return new_x_train

def PALM(x_train):

    x_train = x_train.T
    M       = 32
    L       = 28 * 28
    N       = 30000

    D       = np.random.rand(L, M)
    W       = np.random.rand(M, N)
    lmbd    = 0.001
    Loss    = 0 # initializing 

    while True  : 
        #Update dictionary D
        step_size    = np.power(la.norm(W, 2),-2) #tc-forum post "PALM"
        D_derivative = np.matmul(np.matmul(D, W) - x_train, W.T)
        D = np.subtract(D, step_size * D_derivative) #D = D - (step_size * D_derivative)

        for j in range(0, M):
            if j > 0:
                D[:,j]   = D[:,j] - np.matmul(np.ones(D[:,j].shape).T, D[:,j])/L
            D[:,j]   = D[:,j] / np.maximum(1, la.norm(D[:,j], 2)) 
        
        #Update Wj
        step_size    = np.power(la.norm(D, 2), -2) 
        W_derivative = np.matmul(D.T, np.matmul(D, W) - x_train)
        W            = W - step_size * W_derivative
        for j in range(1, M):
            arny =  np.maximum(0, (np.absolute(W[j]) - lmbd))
            W[j]     = np.multiply(arny, W[j]/np.absolute(W[j]))
        
        #convergence criteria
        diff         = abs(Loss - 1/2 * np.power(la.norm(np.matmul(D,W) - x_train, 'fro'),2 )) # Loss - new Loss
        print('diff ', diff, '  Loss ', Loss)
        if (diff < 1000): # 1000 
            break
        Loss         = 1/2 * np.power(la.norm(np.matmul(D,W) - x_train, 'fro'), 2)

    #1.b
    D_images = []
    for i in range(32):
        D_images.append(np.reshape(D[:,i], (28,28)))
    show_images(np.array(D_images), 4)

    #1.c    from 32 to 40
    Reconstructed = []
    for i in range(32, 40):
        Reconstructed.append(np.reshape(np.matmul(D, W[:,i]), (28,28)))
        Reconstructed.append(np.reshape(x_train[:,i], (28,28)))
    show_images(np.array(Reconstructed), 4)
    return W


########
#TASK 2#
########
learning_rate = 0.05
treshold      = 0.5

def sigmoid(Z):
    return 1 / (1 + np.exp(-Z))

def logistic_entropy_loss(y, log_sig):
    return -(y * np.log(log_sig) + (1-y) * np.log(1-log_sig)).mean() 

loss     = []
thetas   = np.zeros((5, 33))

def classify(W, y_train, theta, new_class):
    y = (y_train == new_class).astype(int) #binary classifier
    for i in range(10):
        for j in range(30000):
            ri = np.random.randint(0,30000)
            z = W.T[ri] @ theta 
            h = sigmoid(z)
            gradient = W[:,ri] * (h - y[ri]) + (theta*0.2)#/ 30000
            theta = (theta - learning_rate * gradient)  
        print("#", end="")
        sys.stdout.flush()
  
    loss.append(logistic_entropy_loss(y, h))
    thetas[new_class, :] = theta

def Logistic(W, y_train):
    #extend W by one row of ones (bias)
    bias = np.ones((1, 30000), dtype=int)
    new_w = np.concatenate((bias, W), axis=0)
    W = new_w
    theta = np.zeros(np.shape(W[:,1]))
    print(theta.shape)
    
    #fit - trian
    for new_class in range(5):
        classify(W, y_train, theta, new_class)
    print ('Loss:     ', np.array(loss).mean())
    print ('Accuracy: ', np.mean(sigmoid(W.T @ thetas.T).argmax(axis=1) == y_train))
    plot.PCA_plot(W, y_train)

def main():
    x_train, y_train = import_data()
    x_train = scale_and_center()
    W = PALM(x_train)
    Logistic(W, y_train)


    
if __name__ == '__main__':
    main()