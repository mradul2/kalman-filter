import numpy as np 
import matplotlib.pyplot as plt
import os 
from math import sqrt 

#------------------------------------ readFile Function --------------------------------------------#

def readFile(filePath: str = None) -> list:
    "returns a list of list of variables in Order of x,y,vx,vy"

    try:
        FILE = open(filePath,'r')
    except:
        print("Unable to open File!")
        exit()

    LINES = FILE.readlines()

    DATA = []

    for LINE in LINES:
        LINESPLIT = LINE.split()
        EVENINDEXED = LINESPLIT[::2]
        FLOAT = [float(STRING) for STRING in EVENINDEXED]
        DATA.append(FLOAT)

    return DATA


#------------------------------------ Global Matrices --------------------------------------------#

# CONTROL MATRICES 
A = np.array([[1,1],[0,1]])
H = np.array([[1,0],[0,1]])
C = np.array([[1,0],[0,1]])

# COVARIANCE MATRICES
Q = np.array([[0.0025,0.00035],[0.000035,0.00025]])
R = np.array([[0.01,0.00035],[0.00035,0.001]])

# ERROR MATRICES
Z = np.array([0.02,0.001])
W = np.array([0.02,0.001])

# FILE PATH
FILEPATH = os.path.join(os.getcwd(),"kalman.txt")

#------------------------------------ prediction Function -----------------------------------------#

def prediction(Xprev: np.ndarray = None, Pprev: np.ndarray = None) -> tuple:
    Xpredicted = A.dot(Xprev) + W
    Ppredicted = (A.dot(Pprev)).dot(A.T) + Q
    return Xpredicted, Ppredicted

#------------------------------------- updation Function -----------------------------------------#

def updation(Xpredicted: np.ndarray = None, Ppredicted: np.ndarray = None, Xmeasured: np.ndarray = None) -> tuple:
    Innovation = C.dot(Xmeasured) + Z
    Covariance = (H.dot(Ppredicted)).dot(H.T) + R
    KalmanGain = (Ppredicted.dot(H.T)).dot(np.linalg.inv(Covariance))

    Xupdated = Xpredicted + KalmanGain.dot(Innovation - H.dot(Xpredicted))
    Pupdated = Ppredicted - (KalmanGain.dot(Covariance)).dot(KalmanGain.T)

    return Xupdated, Pupdated


# Main Driver Function
def main():
    DATA = readFile(FILEPATH)


#--------------------------------------- For X and VX ------------------------------------------#
    X_measured = []
    VX_measured = []

    X_measured.append(DATA[0][0])

    for DATASET in DATA[1:]:
        X_measured.append(DATASET[0])
        VX_measured.append(DATASET[2])

    X_final = []
    X_predicted = []
    VX_final = []
    VX_predicted = []

    X_uncertainity = [0.1]
    VX_uncertainity = [0.01]

    X_final.append(DATA[0][0])
    
    Xprev = np.array([DATA[0][0],0])
    Pprev = np.array([[0.01,0],[0,0.0001]])

    for DATASET in DATA[1:]:

        Xpredicted, Ppredicted = prediction(Xprev, Pprev)
        Xupdated, Pupdated = updation(Xpredicted, Ppredicted, np.array([DATASET[0],DATASET[2]]))

        Xprev = Xupdated
        Pprev = Pupdated

        X_final.append(Xupdated[0])
        X_predicted.append(Xpredicted[0])
        VX_final.append(Xupdated[1])
        VX_predicted.append(Xpredicted[1])

        X_uncertainity.append(sqrt(Pupdated[0][0]))
        VX_uncertainity.append(sqrt(Pupdated[1][1]))
#--------------------------------------- For Y and VY ------------------------------------------#

    Y_measured = []
    VY_measured = []

    Y_measured.append(DATA[0][1])    

    for DATASET in DATA[1:]:
        Y_measured.append(DATASET[1])
        VY_measured.append(DATASET[3])

    Y_final = []
    Y_predicted = []
    VY_final = []
    VY_predicted = []

    Y_uncertainity = [0.1]
    VY_uncertainity = [0.01]

    Y_final.append(DATA[0][1])
    
    Yprev = np.array([DATA[0][1],0])
    Pprev = np.array([[0.01,0],[0,0.0001]])

    for DATASET in DATA[1:]:

        Ypredicted, Ppredicted = prediction(Yprev, Pprev)
        Yupdated, Pupdated = updation(Ypredicted, Ppredicted, np.array([DATASET[1],DATASET[3]]))

        Yprev = Yupdated
        Pprev = Pupdated

        Y_final.append(Yupdated[0])
        Y_predicted.append(Ypredicted[0])
        VY_final.append(Yupdated[1])
        VY_predicted.append(Ypredicted[1])

        Y_uncertainity.append(sqrt(Pupdated[0][0]))
        VY_uncertainity.append(sqrt(Pupdated[1][1]))

#-----------------------------Writing data---------------------------------------------------------#

    fp = open("Beliefstate.txt","w")
    for (x,y,vx,vy) in zip(X_final,Y_final,VX_final,VY_final):
        line = str(x) + "   " + str(y) + "   " + str(vx) + "   " + str(vy) + "\n"
        fp.write(line)
        

    fp = open("Uncertainity.txt","w")
    for (x,y,vx,vy) in zip(X_uncertainity,Y_uncertainity,VX_uncertainity,VY_uncertainity):
        line = str(x) + "   " + str(y) + "   " + str(vx) + "   " + str(vy) + "\n"
        fp.write(line)

#------------------------------------- Plotting Graphs ---------------------------------------------#

    figure, axis = plt.subplots(2, 2)

    "X vs Y Curves"
    
    axis[0,0].plot(X_final,Y_final,color='green')
    axis[0,0].plot(X_measured,Y_measured,color='red')
    axis[0,0].plot(X_predicted,Y_predicted,color='blue')
    axis[0,0].title.set_text("Combined")
    
    axis[0,1].plot(X_final,Y_final,color='green')
    axis[0,1].title.set_text("Final")

    axis[1,1].plot(X_measured,Y_measured,color='red')
    axis[1,1].title.set_text("Measured")

    axis[1,0].plot(X_predicted,Y_predicted,color='blue')
    axis[1,0].title.set_text("Predicted")

    plt.show()


if __name__ == '__main__':
    main()