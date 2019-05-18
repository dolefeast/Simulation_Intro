from kutta_test import rungeKutta
from euler_test import euler
import matplotlib.pyplot as plt
import numpy as np
from time import time
import pandas as pd

def dydx(x, y):
    return y 
def f(x):
    return np.exp(x)

x0 = 1
y0 = np.e 
x = 40
h = 1
X = np.arange(x0, x, h)
print(X)
true_values = f(X+np.log(y0))
rK_values = np.array(rungeKutta(x0, y0, x, h, dydx))
e_values = np.array(euler(x0, y0, x, h, dydx))
print(np.shape(true_values), np.shape(rK_values))
"""
df = pd.DataFrame(X, rK_values)
"""
df = pd.DataFrame(true_values, rK_values)
print(df)


plt.plot(X, e_values, 'c')
plt.plot(X, true_values, 'm')
plt.plot(X, rK_values)
plt.show()
