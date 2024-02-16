import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


fig, ax = plt.subplots()
fig2, ax2 = plt.subplots(subplot_kw={"projection": "3d"})

dt = 0.01
goal =  np.array([5,5])
obs1 = np.array([2.8 ,2.5])
obs2 = np.array([3,4])

def force(pose,goal,obsPose1,obsPose2):
    K = 5
    K2 = 1
    d0 = 3
    d_ob1 =np.linalg.norm(obsPose1-pose)
    d_ob2 =np.linalg.norm(obsPose2-pose)
    return K*(goal-pose) + 2*K2*(1/d_ob1-1/d0)*1/d_ob1**(5/2)*(pose-obsPose1) +  2*K2*(1/d_ob2-1/d0)*1/d_ob2**(5/2)*(pose-obsPose2)
    # return - 2*K2/((np.linalg.norm(obsPose-pose)+1e-3)**3)*np.ones(2)

def PotentialField(pose,goal,obsPose1,obsPose2):
    K = 2
    K2 = 1
    d0 = 3
    return K*np.linalg.norm(goal-pose) + K2*(1/np.linalg.norm(obsPose1-pose)-1/3)**2 + K2*(1/np.linalg.norm(obsPose2-pose)-1/3)**2
    # return - 2*K2/((np.linalg.norm(obsPose-pose)+1e-3)**3)*np.ones(2)
def plotpotential():

    X=np.linspace(2,4,100)
    Y=np.linspace(2,4,100)
    X, Y = np.meshgrid(X, Y)
    Z=np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i,j] = min(50.,PotentialField(np.array([X[i,j],Y[i,j]]),goal,obs1,obs2))
    surf = ax2.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
    plt.show()

def plotQuiver():
    goal =  np.array([5,5])
    obs = np.array([2.5,2.5])
    x = np.arange(0,5.2,0.2)
    y = np.arange(0,5.2,0.2)
    X, Y = np.meshgrid(x, y)
    F=[]
    for i,j in zip(X,Y):
        for p,q in zip(i,j):
            print (p,q)
            F.append(force(np.array([p,q]),goal,obs))
    F=np.array(F)
    ax.quiver(X,Y,F[:,0],F[:,1])

    ax.xaxis.set_ticks([])
    ax.yaxis.set_ticks([])
    ax.axis([-0.2, 5.2, -0.2, 5.2])
    ax.set_aspect('equal')

    plt.show()
def pointmass(x,u):
    return x + u*dt + 0.02*np.random.randn(2)
def main():
    noSamples = 20
    for j in range(noSamples):
        follower = np.array([2,2])
        _X,_U,_C=[],[],[]
        for i in range(80):
            u = force(follower,goal,obs1,obs2)
            c = PotentialField(follower,goal,obs1,obs2)
            print (i,u)
            follower = pointmass(follower,u)
            print ("follower",follower[0],follower[1])
            _X.append(follower)
            _U.append(u)
            _C.append(c)
            # if i%5==0:
            #     plt.figure(3)
            #     plt.cla()
            #     plt.scatter(follower[0],follower[1])
            #     plt.scatter(obs1[0],obs1[1])
            #     plt.scatter(obs2[0],obs2[1])
            #     plt.scatter(goal[0],goal[1])
            #     plt.axis([0,6,0,6])
            #     plt.pause(1e-6)
        if j==0:
            X=np.array(_X)
            U=np.array(_U)
            C=np.array(_C).reshape(-1,1)
        else:
            X=np.append(X,np.array(_X),axis=1)
            U=np.append(U,np.array(_U),axis=1)
            C=np.append(C,np.array(_C).reshape(-1,1),axis=1)

    plt.figure(4)
    for k in range(noSamples):
        plt.plot(X[:,k*2],X[:,k*2+1])
    plt.show()
    np.savetxt("xTraj",X)
    np.savetxt("uTraj",U)
    np.savetxt("cTraj",C)
if __name__=="__main__":
    main()
    # plotpotential()
    plotQuiver()
