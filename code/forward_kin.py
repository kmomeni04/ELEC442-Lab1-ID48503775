import numpy as np

L_1 = 0.1400
L_2 = 0.3500
L_3 = 0.0500
L_4 = 0.2500
L_5 = 0.1500
BETA = np.arctan(L_3/L_2)

LAMBDA_1 = L_1
LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
LAMBDA_3 = L_4 + L_5

def dh_transform(theta, d, a, alpha):
    '''
    Standard DH homogeneous transform from frame {i} to frame {i-1}:
    T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    '''
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])

def forward_kin(phi):
    '''
    Implement a forward kinematics function for the QArm

    Input:
        phi: the joint angles in radians, in order of base, shoulder, elbow, wrist

    Output:
        location: the end effector coordinate system origin position in the base {0} coordinate system
        rotation: rotation matrix from the end effector {4} coordinate system to the base {0} coordinate system
    '''

    phi = np.array(phi, dtype=float, copy=True)

    # == your code here ==

    # DH parameters for each link: theta, d, a, alpha
    # TODO: fill in the correct values (theta usually includes phi[i] plus any offset, e.g. BETA)
    T01 = dh_transform(theta=phi[0], d=LAMBDA_1, a=0.0, alpha=np.radians(-90))  # base -> {1}
    T12 = dh_transform(theta=phi[1]-np.radians(90)+BETA, d=0.0, a=LAMBDA_2, alpha=0.0)  # {1} -> {2}
    T23 = dh_transform(theta=phi[2]-BETA, d=0.0, a=0.0, alpha=np.radians(-90))  # {2} -> {3}
    T34 = dh_transform(theta=phi[3], d=LAMBDA_3, a=0.0, alpha=0.0)  # {3} -> {4} end effector

    # Transform from end effector {4} to base {0}
    T04 = T01 @ T12 @ T23 @ T34

    location = T04[:3, 3]
    rotation = T04[:3, :3]

    # == end of your code ==
    location[np.abs(location) < 1e-10] = 0.0
    rotation[np.abs(rotation) < 1e-10] = 0.0

    return location, rotation



if __name__ == '__main__':

    # Function input is radians

    phi = np.radians(np.array([0, 0, 0, 0]))
    location, rotation = forward_kin(phi)
    print('Input 1 -> location:', location, 'rotation:', rotation)
    
    phi = np.radians(np.array([30, 55, -60, 0]))
    location, rotation = forward_kin(phi)
    print('Input 2 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([0, -11, -75, 0]))
    location, rotation = forward_kin(phi)
    print('Input 3 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([-90, 73.5, -75, -50]))
    location, rotation = forward_kin(phi)
    print('Input 4 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([-90, 73.5, -75, 0]))
    location, rotation = forward_kin(phi)
    print('Input 5 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([90, -85, 75, 90]))
    location, rotation = forward_kin(phi)
    print('Input 6 -> location:', location, 'rotation:', rotation)


