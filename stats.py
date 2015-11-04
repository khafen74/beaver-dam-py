import numpy as np

def getDamHeightLognormal(mu, sigma):
    rn = np.random.lognormal(mu, sigma, 1)
    return rn