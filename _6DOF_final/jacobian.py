import sympy as sp

L1 =0.1079 #could also be 0.04
L2 = 0
L3 = 0.1397
L4 = 0.3302
L5 = 0.3302
L6 = 0.0762
L7 = 0.0635
L8 = 0.0826
L9 = 0.0381
L10 = 0.05

def AN1(theta):
    mat = sp.Matrix([[sp.cos(theta), -sp.sin(theta),0,0],
                        [sp.sin(theta),sp.cos(theta),0,0],
                        [0,0,1,L1],
                        [0,0,0,1]])
    return mat


def A12(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, sp.sin(theta), L2],
                        [0,1,0,0],
                        [-sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A23(theta):
    mat = sp.Matrix([[1, 0, 0, L3],
                        [0,sp.cos(theta),-sp.sin(theta),0],
                        [0,sp.sin(theta),sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A34(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, sp.sin(theta), L4],
                        [0,1,0,-L7],
                        [-sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat


def A45(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, sp.sin(theta), L5],
                        [0,1,0,-L8],
                        [-sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A56(theta):
    mat = sp.Matrix([[1, 0, 0, L6+L10],
                        [0,sp.cos(theta),-sp.sin(theta),-L9],
                        [0,sp.sin(theta),sp.cos(theta),0],
                        [0,0,0,1]])
    return mat
# def A6E(theta):
#     mat = sp.Matrix([[1, 0, 0, L10],
#                         [0,1,0,0],
#                         [0,0,1,0],
#                         [0,0,0,1]])
#     return mat

def AttndPos(thetas):
    mat1 = AN1(thetas[0,0])
    mat2 = A12(thetas[1,0])
    mat3 = A23(thetas[2,0])
    mat4 = A34(thetas[3,0])
    mat5 = A45(thetas[4,0])
    mat6 = A56(thetas[5,0])

    AIE = mat1*mat2*mat3*mat4*mat5*mat6
    AIE.row_del(3)

    a = AIE.col(0)
    o = AIE.col(1)
    n = AIE.col(2)
    p = AIE.col(3)
    
    f = sp.Matrix([[a],[o],[n],[p]])

    AIE.col_del(3)
    CIE = AIE


    return (a,o,n,p,f) 

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

def bigJ(angv,a,o,n,p):
    J1 = jacobi(a,angv)
    J2 = jacobi(o,angv)
    J3 = jacobi(n,angv)
    J4 = jacobi(p,angv)
    J = sp.Matrix([[J1],[J2],[J3],[J4]])
    return J

def FirstJ():

    tht1,tht2,tht3,tht4,tht5,tht6 = sp.symbols('tht1 tht2 tht3 tht4 tht5 tht6', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3],[tht4],[tht5],[tht6]])
    Forwards = AttndPos(angs)
    avec = Forwards[0]
    ovec = Forwards[1]
    nvec = Forwards[2]
    pvec = Forwards[3]    
    Jsym = bigJ(angs,avec,ovec,nvec,pvec)
    return Jsym


def bigSub():
    J = FirstJ()
    tht1,tht2,tht3,tht4,tht5,tht6 = sp.symbols('tht1 tht2 tht3 tht4 tht5 tht6', real = True)
    Jfunk = sp.lambdify([tht1,tht2,tht3,tht4,tht5,tht6],J)
    return Jfunk

