import numpy as np
from matplotlib import pyplot as plt
from sklearn.decomposition import PCA


def PCA_plot(W, y, size=(-4, 4)):
    """
    M is the number of dictionary atoms, N is the number of samples in the dataset.
    :param W: [M x N] Matrix containing dictionary encodings
    :param y: [N] Vector containing the labels
    :param size: tuple defining the range of the x and y axes for plotting
    """

    pca = PCA(2)
    W_red = pca.fit_transform(W.T)

    fig, ax = plt.subplots()
    idx = slice(0, 1000, 1)
    color_labels = _replace_class_labels(y[idx])
    ax.scatter(W_red[idx, 0], W_red[idx, 1], c=color_labels, s=30, edgecolor='k')
    ax.set(xlim=size, ylim=size)
    plt.show()

def _replace_class_labels(yy, n_classes=5):
    y = np.copy(yy)
    idxs = []
    new_classes = np.arange(n_classes)
    classes = np.sort(np.unique(y))
    for i in range(n_classes):
        idx = np.where(y == classes[i])
        idxs.append(idx)

    for i in range(n_classes):
        idx = idxs[i]
        y[idx] = new_classes[i]

    return y

