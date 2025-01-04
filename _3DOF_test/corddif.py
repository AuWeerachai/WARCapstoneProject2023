import numpy as np

def AN1(theta):
    mat = np.matrix([[np.cos(theta), -np.sin(theta),0,0],
                        [np.sin(theta),np.cos(theta),0,0],
                        [0,0,1,0.05],
                        [0,0,0,1]])
    return mat


def A12(theta):
    mat = np.matrix([[np.cos(theta), 0, -np.sin(theta), 0],
                        [0,1,0,-0.07],
                        [np.sin(theta),0,np.cos(theta),0.07],
                        [0,0,0,1]])
    return mat

def A23(theta):
    mat = np.matrix([[np.cos(theta), 0, -np.sin(theta), 0.33],
                        [0,1,0,-0.07],
                        [np.sin(theta),0,np.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A34():
    mat = np.matrix([[1, 0, 0, 0.33],
                        [0,1,0,-0.07],
                        [0,0,1,0],
                        [0,0,0,1]])
    return mat
    

def AttndPos(theta1, theta2, theta3):
    mat1 = AN1(theta1)
    mat2 = A12(theta2)
    mat3 = A23(theta3)
    mat4 = A34()

    AIE = np.matmul(np.matmul(mat1,mat2,mat3), mat4)

    a = AIE[:,0]
    o = AIE[:,1]
    n = AIE[:,2]
    p = AIE[:,3]
    
    a = np.delete(a,-1)
    o = np.delete(o,-1)
    n = np.delete(n,-1)
    p = np.delete(p,-1)

    CIE = np.concatenate((np.transpose(a),np.transpose(o),np.transpose(n),), axis = 1)
    f = np.transpose(np.concatenate((a,np.concatenate((o,n,p), axis = 1)), axis = 1))

    return (f,CIE) 

fmoment = AttndPos(1,0,0)[0]
CIEmoment = AttndPos(1,0,0)[1]
print(fmoment)
print(CIEmoment)