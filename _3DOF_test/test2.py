import numpy as np
import sympy as sp

def AN1(theta):
    mat = sp.Matrix([[sp.cos(theta), sp.sin(theta),0,0],
                        [-sp.sin(theta),sp.cos(theta),0,0],
                        [0,0,1,0.05],
                        [0,0,0,1]])
    return mat


def A12(theta):
    mat = sp.Matrix([[-sp.sin(theta), 0, sp.cos(theta), 0],
                        [sp.cos(theta),0,sp.sin(theta),-0.07],
                        [0,1,0,0.07],
                        [0,0,0,1]])
    return mat

def A23(theta):
    mat = sp.Matrix([[sp.cos(theta), sp.sin(theta),0, 0.33],
                        [-sp.sin(theta),sp.cos(theta),0,-0.07],
                        [0,0,1,0],
                        [0,0,0,1]])
    return mat

def A34():
    mat = sp.Matrix([[1, 0, 0, 0.33],
                        [0,1,0,-0.05],
                        [0,0,1,0],
                        [0,0,0,1]])
    return mat

    

def jacobi(b,thetas):
    r1 = []
    r2 = []
    r3 = []
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[0],thetas[i])
        r1.append(var)
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[1],thetas[i])
        r2.append(var)
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[2],thetas[i])
        r3.append(var)
    r1 = sp.Matrix([r1])
    r2 = sp.Matrix([r2])
    r3 = sp.Matrix([r3])

    Jn = sp.Matrix([[r1],[r2],[r3]])
    return (Jn)


def AttndPos(theta1, theta2, theta3):
    mat1 = AN1(theta1)
    mat2 = A12(theta2)
    mat3 = A23(theta3)
    mat4 = A34()

    AIE = mat1*mat2*mat3*mat4
    AIE.row_del(3)

    a = AIE.col(0)
    o = AIE.col(1)
    n = AIE.col(2)
    p = AIE.col(3)
    
    f = sp.Matrix([[a],[o],[n],[p]])

    AIE.col_del(3)
    CIE = AIE


    return (a,o,n,p,f,CIE) 

def bigJ(angv,a,o,n,p):
    J1 = jacobi(a,angv)
    J2 = jacobi(o,angv)
    J3 = jacobi(n,angv)
    J4 = jacobi(p,angv)
    J = sp.Matrix([[J1],[J2],[J3],[J4]])
    return J

def tonumpy(m):
    numrows = sp.shape(m)[0]
    numcols =sp.shape(m)[1]
    nummy = np.zeros([numrows, numcols])
    for i in range(0,numcols):
        for j in range(0,numrows):
            nummy[j,i] = m[j,i]

    return nummy

#define our actuator angles as a vector of symbols
def calcX(thetaActual,desiredXYZ):

    tht1,tht2,tht3 = sp.symbols('tht1 tht2 tht3', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3]])
    #Determine forward Kinematics Parameters
    Forwards = AttndPos(tht1,tht2,tht3)
    avec = Forwards[0]
    ovec = Forwards[1]
    nvec = Forwards[2]
    pvec = Forwards[3]
    fmoment = Forwards[4]
    CIEmoment = Forwards[5]
    #Find the Jacobian of the matrix symbolically
    Jsym = bigJ(angs,avec,ovec,nvec,pvec)
    #Start Subbing in the real values from feedback
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    angs = np.matrix([[ang1],[ang2],[ang3]])
    J = Jsym.subs(tht1,ang1)
    J = J.subs(tht2, ang2)
    J = J.subs(tht3,ang3)
    fold = fmoment.subs(tht1,ang1)
    fold = fold.subs(tht2,ang2)
    fold = fold.subs(tht3,ang3)
    fnew = np.zeros([12,1])
    fnew[0]=fold[0]
    fnew[1]=fold[1]
    fnew[2]=fold[2]
    fnew[3]=fold[3]
    fnew[4]=fold[4]
    fnew[5]=fold[5]
    fnew[6]=fold[6]
    fnew[7]=fold[7]
    fnew[8]=fold[8]
    fnew[9]= desiredXYZ[0]
    fnew[10]= desiredXYZ[1]
    fnew[11]= desiredXYZ[2]
    #put into numpy for easier calcs
    J = tonumpy(J)
    fold = tonumpy(fold)
    fnew = tonumpy(fnew)
    # do the actual calcs of X
    invJ = np.linalg.pinv(J)
    X = np.matmul(invJ,(fnew - fold))
    return X


def checkforexit(tht,cmnd):
    check = abs(tht - cmnd)
    # print(check)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol)

def allcalcs(XYZ):
    thtfeedback = np.matrix([[0.0],[0.0],[0.0]])
    cmnd = thtfeedback
    loop = True

    anglepath = np.matrix([0,0,0])
    while loop:
        X = calcX(cmnd,XYZ)
        thtnew = cmnd + X
        logger = np.transpose(thtnew)
        anglepath = np.append(anglepath,logger,axis = 0)
        if checkforexit(thtnew,cmnd):
            loop = False
        
        else:
            cmnd = thtnew

    anglepath = np.delete(anglepath, (0), axis=0)
    anglepath = np.transpose(anglepath)
    print("Done Calculating Path")
    return(anglepath)


def calculatedXYZ(finalangles):
    f = AttndPos(finalangles[0,0],finalangles[1,0],finalangles[2,0])[4]
    f = np.matrix([[f[9]],[f[10]],[f[11]]])
    return f


tol = 0.005
XYZ = np.matrix([[0.4],[0.35],[-0.45]]) #where we wanna go XYZ
pos = allcalcs(XYZ)
final = pos[:,(np.shape(pos)[1]-1)]
newXYZ = calculatedXYZ(final)
diff = XYZ - newXYZ
print(XYZ)
print(newXYZ)
print(pos[:,-1]) #print final set of gimbal angles
