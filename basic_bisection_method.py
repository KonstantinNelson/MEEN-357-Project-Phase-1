import numpy as np

def basic_bisection(fun, xl, xu, err_max=1e-5, iter_max=1000): #5 inputs
    
    # initialize number of iterations
    numIter = 0
    
    # initialize Boolean variable
    done = False
    
    # initialize estimated error 
    err_est = np.nan #NumPy NAN stands for not a number and is defined as a substitute for declaring 
                     #value which are numerical values that are missing values in an array
    root = np.nan  #in case it is never established
    
    # MAIN LOOP
    while not done: #while the variable done is NOT equal True, to keep the While loop running until done = True
        numIter += 1    #means numIter = numIter + 1 #add 1 more to the variable numIter
        
        xr = (xu+xl)/2
        
        # fun is a function defined elsewhere, a function of x to be solved for root 
        if fun(xl)*fun(xr) < 0: #this happen if signs are different
            xu = xr
        else:
            xl = xr
        
        #if fun(xl)*fun(xr) > 0: #this happen if signs are the same
        #    xl = xr
        #else:
        #    xu = xr
        
        # update estimated error
        err_est = 100*abs((xu-xl)/(xr))
        
        if err_est < err_max:
            done = True
            root = xr
            exitFlag = 1    #1 if algorithm terminates properly
        elif numIter >= iter_max:    #compare with else: above!
            done = True
            root = xr   # this is not necessarily a root though! Could be just a closest guess
            exitFlag = 0    #0 if max iteration value is reached
    
    # REPORT OUT
    return root, err_est, numIter, exitFlag #4 outputs
