import matplotlib.pyplot as plt

def plotVector(vector, color):
    """
    Parameters
    ----------
    vector: array-like, length 2
        x-y component vector to plot
    color: string
        color of arrow to plot. like 'k' for black or 'r' for red
    """
    X, Y = (0, 0)
    U, V = (vector[0], vector[1])
    plt.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1,color=color)
    plt.xlim([-1.1,1.1])
    plt.ylim([-1.1,1.1])
