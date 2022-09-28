import numpy as np

def basic_bisection(fun,xl,xu,err_max=1e-5,iter_max=1000):
    
    if not callable(fun):
        raise Exception('The first input must be a callable function')
    if not isinstance(xl,float) and not isinstance(xl,int):
        raise Exception('The second and third inputs must be scalar values')
    if not isinstance(xu,float) and not isinstance(xu,int):
        raise Exception('The second and third inputs must be scalar values')
   
    numIter=0
    done=False
    err_est=np.nan #error estimate
    root=np.nan #value of x where function equals 0
    while not done:
        numIter +=1
        xr=(xu+xl)/2 #approximation of root
        if fun(xl)*fun(xr)<0: #fun is function of x to be solved
            xu=xr
        else:
            xl=xr
        err_est=100*abs((xu-xl)/(xr)) #update estimated error
        if err_est < err_max:
            done=True
            root=xr
            exitFlag=1 #1 if algorithm terminates properly
        elif numIter >= iter_max:
            done=True
            root=xr #not necessarily the root
            exitFlag=0 #0 if temrination value is reached
    return root, err_est,numIter,exitFlag
