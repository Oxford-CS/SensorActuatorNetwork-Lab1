import math
import numpy as np


def log_path_model(dist):
    # calibrate on 1m
    n = 1.276
    rss_ref = -32
    rss = rss_ref - 10*n*math.log10(dist) + np.random.normal(0, 1, 1)
    return rss