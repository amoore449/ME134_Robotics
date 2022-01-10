import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression

def fit_func(data, inter, coef): #Cubic surface function
    x = data[0]
    y = data[1]
    return inter + coef[0] + coef[1] * y + coef[2]*y**2 + coef[3]*y**3 + coef[4]*x + coef[5]*x*y + coef[6]*x*y**2 + coef[7]*x*y**3 + coef[8]*x**2 + coef[9]*x**2*y + coef[10]*x**2*y**2 + coef[11]*x**2*y**3 + coef[12]*x**3 + coef[13]*x**3*y + coef[14]*x**3*y**2 + coef[15]*x**3*y**3

def surface_approximation(n_samples, Z_data, xbounds, ybounds):
    #Create an array of points given input x and y bounds
    x = np.linspace(xbounds[0], xbounds[1], n_samples).reshape(n_samples, 1)
    y = np.linspace(ybounds[0], ybounds[1], n_samples).reshape(1, n_samples)
    #Create feature dictionary
    features = {}
    features['x^0*y^0'] = np.matmul(x**0, y**0).flatten()
    features['x^0*y'] = np.matmul(x**0, y).flatten()
    features['x^0*y^2'] = np.matmul(x**0, y**2).flatten()
    features['x^0*y^3'] = np.matmul(x**0, y**3).flatten()
    features['x*y^0'] = np.matmul(x, y**0).flatten()
    features['x*y'] = np.matmul(x, y).flatten()
    features['x*y^2'] = np.matmul(x, y**2).flatten()
    features['x*y^3'] = np.matmul(x, y**3).flatten()
    features['x^2*y^0'] = np.matmul(x**2, y**0).flatten()
    features['x^2*y'] = np.matmul(x**2, y).flatten()
    features['x^2*y^2'] = np.matmul(x**2, y**2).flatten()
    features['x^2*y^3'] = np.matmul(x**2, y**3).flatten()
    features['x^3*y^0'] = np.matmul(x**3, y**0).flatten()
    features['x^3*y'] = np.matmul(x**3, y).flatten()
    features['x^3*y^2'] = np.matmul(x**3, y**2).flatten()
    features['x^3*y^3'] = np.matmul(x**3, y**3).flatten()

    data = pd.DataFrame(features) #Make a dataframe of all polynomial features

    reg = LinearRegression().fit(data.values, Z_data.flatten()) #Compute a linear regression to get polynomial coefficients

    return reg.intercept_, reg.coef_