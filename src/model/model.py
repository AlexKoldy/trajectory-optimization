from pyquaternion import Quaternion


omega_max = 5.5
T_r = -36.07956616966136 # torque coefficient for roll
T_p = -12.14599781908070 # torque coefficient for pitch
T_y =   8.91962804287785 # torque coefficient for yaw
D_r =  -4.47166302201591 # drag coefficient for roll
D_p = -2.798194258050845 # drag coefficient for pitch
D_y = -1.886491900437232 # drag coefficient for yaw

class Conditions():
    def __init__(self, s=Vector3(0,0,0), v_mag=0.0, r=Rotator(0,0,0), w=Vector3(0,0,0), bs=Vector3(0,0,0), bv=Vector3(0,0,0), br=Rotator(0,0,0), bw=Vector3(0,0,0)):
        # Set car state stuff
        self.s = s
        self.v_mag =v_mag
        self.r = r
        self.w = w
        yaw = r.yaw
        vx = v_mag * np.cos(yaw)
        vy = v_mag * np.sin(yaw)
        self.v = Vector3(x=vx, y=vy, z=0.0)

        self.bs = bs
        self.bv = bv
        self.br = br
        self.bw = bw


class AerialConditions():
    def __init__(self, s=Vector3(0,0,0), v=Vector3(0,0,0), r=Rotator(0,0,0), w=Vector3(0,0,0), bs=Vector3(0,0,0), bv=Vector3(0,0,0), br=Rotator(0,0,0), bw=Vector3(0,0,0)):
        # Set car state stuff
        self.s = s
        self.v = v
        self.r = r
        self.w = w
        _, self.q = AerialConditions.euler_to_left_handed_rotation_and_quaternion(r)

        self.bs = bs
        self.bv = bv
        self.br = br
        self.bw = bw

    #TODO: Needed one that had velocity as magnitude, and needed one that had velocity in component form
    @staticmethod
    def build_initial_from_initial_conditions_object(ic: InitialConditionsGekko):
        i = ic.params
        s = Vector3(float(i['sxi']), float(i['syi']), float(i['szi']))
        v = Vector3(float(i['vx']), float(i['vy']), float(i['vz']))
        r = Rotator(roll=float(i['rolli']), pitch=float(i['pitchi']), yaw=float(i['yawi']))
        bs = Vector3(float(i['bxi']), float(i['byi']), float(i['bzi']))
        bv = Vector3(float(i['bvxi']), float(i['bvyi']), float(i['bvzi']))
        condit = AerialConditions(s=s, v=v, r=r, bs=bs, bv=bv)
        return condit

    @staticmethod
    def build_final_from_initial_conditions_object(fc: InitialConditionsGekko):
        i = fc.params
        s = Vector3(float(i['sxf']), float(i['syf']), float(i['szf']))
        v = float(i['v_magf'])
        r = Rotator(float(i['rollf']), float(i['pitchf']), float(i['yawf']))
        condit = AerialConditions(s=s, v=v, r=r)
        return condit

    @staticmethod
    def euler_to_left_handed_rotation_and_quaternion(orientation: Rotator):
        r = -1*orientation.roll #rotation around roll axis to get car to world frame
        p = -1*orientation.pitch #rotation around pitch axis to get car to world frame
        y = orientation.yaw #rotation about the world z axis to get the car to the world frame
        Rx = np.matrix([[1, 0, 0], [0, np.cos(r), np.sin(r)], [0, -1*np.sin(r), np.cos(r)]])
        Ry = np.matrix([[np.cos(p), 0, -1*np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]])
        Rz = np.matrix([[np.cos(y), np.sin(y), 0], [-1*np.sin(y), np.cos(y), 0], [0, 0, 1]])
        #Order of rotations from car to world is z then y then x
        Rinter = np.matmul(Rz, Ry)
        Rcar_to_world = np.matmul(Rinter, Rx)

        return Rcar_to_world, Quaternion(matrix=Rcar_to_world)

