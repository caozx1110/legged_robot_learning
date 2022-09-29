from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import numpy as np
import math
from math import sin,cos
from cvxopt import matrix,sparse,solvers
from scipy.optimize import linprog
from scipy.io import savemat
import copy

os.chdir('./project/qz2_mujoco/qz2_controller')

def cross(pos):
    '''
    计算叉乘矩阵
    pos : is a list
    '''
    a3 = pos[2]
    a2 = pos[1]
    a1 = pos[0]
    return np.array([[0, -a3, a2],[a3, 0, -a1],[-a2, a1, 0]])

def quaternion2axis(qua):
    '''
    qua: [cos(the/2),sin(the/2)(x,y,z)]
    return : w*theta in list
    '''
    if qua[0] == 1:
        return [0,0,0]
    else:
        the = 2 * math.acos(qua[0])
        den = math.sqrt(1-qua[0]**2)
        x = qua[1]/den*the
        y = qua[2]/den*the
        z = qua[3]/den*the
        return [x,y,z]

def rotation2axis(Rot):
    the = math.acos((Rot[0,0]+Rot[1,1]+Rot[2,2]-1)/2)
    if math.sin(the) != 0:
        return 0.5/math.sin(the)*np.array([Rot[2,1]-Rot[1,2],Rot[0,2]-Rot[2,0],Rot[1,0]-Rot[0,1]])
    else:
        return np.array([0,0,0])

def quaternion2rotation(qua):
    '''
    qua: quaternion
    return : rotation matrix in np.array
    '''
    w = qua[0]
    x = qua[1]
    y = qua[2]
    z = qua[3]
    row1 = [1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w]
    row2 = [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w]
    row3 = [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]
    return np.array([row1,row2,row3])



class qzrobot():
    def __init__(self,sim):

        self.mass = 38.6               # 总质量
        self.I = np.zeros((3,3))       # body在局部坐标系的惯性张量阵
        self.I[0,0] = 0.432
        self.I[1,1] = 0.5944
        self.I[2,2] = 0.9821

        self.sim = sim    # mujoco sim handler

        self.Jv = [[] for _ in range(6)]      # 各条腿在body系下的Jacobian-v
        self.Jw = [[] for _ in range(6)]      # 各条腿在body系下的Jacobian-w
        self.Rbody = quaternion2rotation(self.sim.data.qpos[3:7]) # body相对于全局坐标的旋转矩阵
        self.Pbody = self.sim.data.qpos[0:3]                      # body相对于全局坐标的位置
        self.hip_pos = [[0.51,0,0],[0.254,-0.215,0],[-0.254,-0.215,0],[-0.51,0,0],[-0.254,0.215,0],[0.254,0.215,0]] # 各腿hip关节在base坐标系下的位置
        self.nomial_foothold = [[0.51,0,-0.43],[0.254,-0.215,-0.43],[-0.254,-0.215,-0.43],[-0.51,0,-0.43],[-0.254,0.215,-0.43],[0.254,0.215,-0.43]] # 各腿足端的相对位置

        self.f = np.zeros((18,1)) # 存储传感器结果,全局坐标系

        self.ctime = 0  # 当前时间

        self.qvel = np.zeros(18)  # 存储关节速度
        self.blinvel = np.zeros(3) # 存储body线速度
        self.bangvel = np.zeros(3) # 存储body角速度
        self.footpos = [np.zeros(3) for _ in range(6)] # 存储各腿末端位置 在全局坐标系

    def initialize(self):

        # 总时间初始化
        self.ctime = 0


        # 读取传感器数据
        self.Sensor()  


    def Step(self):
        '''
        每步更新
        '''
        # 更新状态
        self.Sensor()

        # Balance Control

        # Swing Control


    def Jacobian(self):
        '''
        caculate jacobian of every leg in body frame
        '''
        for i in range(6):
            if i == 0 or i == 1 or i == 5:
                a0 = self.sim.data.qpos[int(7+3*i)]
                a1 = self.sim.data.qpos[int(8+3*i)]
                a2 = self.sim.data.qpos[int(9+3*i)]
                Jhb = np.zeros((3,3))
                Jhb[0,0] = 0
                Jhb[0,1] = -0.2833*cos(a1+a2)-0.3*cos(a1)
                Jhb[0,2] = -0.2833*cos(a1+a2)
                Jhb[1,0] = 0.2833*cos(a0)*cos(a1+a2)+0.3*cos(a0)*cos(a1)+0.08*cos(a0)
                Jhb[1,1] = -0.3*sin(a0)*sin(a1)-0.2833*sin(a0)*sin(a1+a2)
                Jhb[1,2] = -0.2833*sin(a0)*sin(a1+a2)
                Jhb[2,0] = 0.2833*sin(a0)*cos(a1+a2)+0.3*sin(a0)*cos(a1)+0.08*sin(a0)
                Jhb[2,1] = 0.3*sin(a1)*cos(a0)+0.2833*cos(a0)*sin(a1+a2)
                Jhb[2,2] = 0.2833*cos(a0)*sin(a1+a2)

                self.Jv[i] = Jhb

            elif i == 2 or i == 3 or i == 4:
                a0 = self.sim.data.qpos[int(7+3*i)]
                a1 = self.sim.data.qpos[int(8+3*i)]
                a2 = self.sim.data.qpos[int(9+3*i)]
                Jhb = np.zeros((3,3))
                Jhb[0,0] = 0
                Jhb[0,1] = -0.2833*cos(a1+a2)-0.3*cos(a1)
                Jhb[0,2] = -0.2833*cos(a1+a2)
                Jhb[1,0] = 0.2833*cos(a0)*cos(a1+a2)+0.3*cos(a0)*cos(a1)+0.08*cos(a0)
                Jhb[1,1] = -0.3*sin(a0)*sin(a1)-0.2833*sin(a0)*sin(a1+a2)
                Jhb[1,2] = -0.2833*sin(a0)*sin(a1+a2)
                Jhb[2,0] = 0.2833*sin(a0)*cos(a1+a2)+0.3*sin(a0)*cos(a1)+0.08*sin(a0)
                Jhb[2,1] = 0.3*sin(a1)*cos(a0)+0.2833*cos(a0)*sin(a1+a2)
                Jhb[2,2] = 0.2833*cos(a0)*sin(a1+a2)

                rot = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
                self.Jv[i] = np.dot(rot,Jhb)

    def Sensor(self):
        '''
        更新机器人状态
        更新足末端力传感器，全局坐标 self.f
        更新关节速度，self.qvel
        更新body线速度,self.blinvel
        更新body角速度,self.bangvel
        更新足末端位置,全局坐标,self.footpos
        '''
        sensordata = self.sim.data.sensordata
        for i in range(6):
            quat = self.sim.data.get_body_xquat('L'+str(i)+'_foot')
            rot = quaternion2rotation(quat)  # 全局坐标系到foot
            # Rot = np.dot(self.Rbody.T,rot)   # body坐标系到foot
            force = -sensordata[int(3*i):int(3*i+3)] # 机器人受到的地面反作用力
            self.f[int(3*i):int(3*i+3),0] = np.dot(rot,force.reshape(3,1)).reshape(3)

            for j in range(3):
                idx = int(3*i+j+18)
                self.qvel[int(3*i+j)] = sensordata[idx]
        self.blinvel = sensordata[36:39]
        self.bangvel = sensordata[39:42]

        for j in range(6):
            self.footpos[j] = sensordata[int(42+3*j):int(45+3*j)]

        # 更新body状态
        self.Pbody = self.sim.data.qpos[0:3]
        self.Rbody = quaternion2rotation(self.sim.data.qpos[3:7])
        # 更新雅可比阵
        self.Jacobian()

    def InverseKinematics(self,pos):
        '''
        pos: desired foot pos in local hip frame.
        forward kinematics :
            x = -b*sin(t1)-c*sin(t1+t2)
            y = (a+b*cos(t1)+c*cos(t1+t2))*sin(t0)
            z = (-a-b*cos(t1)-c*cos(t1+t2))*cos(t0)
        使用前需考虑超界情况
        '''
        a = 0.08
        b = 0.3
        c = 0.2833
        x = pos[0]
        y = pos[1]
        z = pos[2]

        t0 = math.atan(-y/z) # hip joint
        L = -z/math.cos(t0) - a
        if (L*L+x*x-b*b-c*c)/(2*b*c) > 1:
            print('WARNING:Inverse kinematics no solution at pos:',pos)
            t2 = 0
            return 0,0,0
        elif (L*L+x*x-b*b-c*c)/(2*b*c) < -1:
            print('WARNING:Inverse kinematics no solution at pos:',pos)
            t2 = math.pi
            return 0,0,0
        else:
            t2 = math.acos((L*L+x*x-b*b-c*c)/(2*b*c)) # lower joint 存在两种解
        # 求t1:利用辅助角公式
        A = -b-c*math.cos(t2)
        B = -c*math.sin(t2)
        cphi = A/math.sqrt(A*A+B*B)
        sphi = B/math.sqrt(A*A+B*B) 
        phi = math.atan2(sphi,cphi)

        q = math.asin(x/math.sqrt(A*A+B*B))
        # 
        t1 = (q-phi)%(2*math.pi)
        if abs(b*math.cos(t1)+c*math.cos(t1+t2) - L) > 0.01:
            t1 = (math.pi-q-phi)%(2*math.pi)
            if abs(b*math.cos(t1)+c*math.cos(t1+t2) - L) > 0.01:
                print('WARNING:Inverse kinematics no solution at pos:',pos)
                return 0,0,0
        return t0,t1,t2


        '''
        每个腿在local hip frame的 foothold limit: [0.35,0.15]在高度0.5时
        i : ith leg
        pos : ith leg footpos in local hip frame
        delta : 计算冲量的最小时间间隔
        direction: 计算冲量的方向,在body坐标系下
        v_body: body的线速度，在全局坐标系下
        '''
        # 计算在local hip frame下的footpos 速度
        v = np.dot(self.Rbody.T , v_body.reshape(3,1))
        if i == 2 or i == 3 or i == 4:
            v = np.dot(np.array([[-1,0,0],[0,-1,0],[0,0,1]]),v)
        v = -v
        v = v.reshape(3)
        print('v:',v)

        H_list = []
        b_list = []
        k = 0
        while True:
            t0,t1,t2 = self.InverseKinematics(delta*k*v + pos)
            if (t0 == 0 and t1 == 0 and t2 == 0) or k >= 5:
                break
            else:
                k += 1
                # 统一在Body坐标系下计算
                J_tmp = GenerateJ(t0,t1,t2)
                if i == 2 or i == 3 or i == 4:
                    J_tmp = np.dot(np.array([[-1,0,0],[0,-1,0],[0,0,1]]),J_tmp)
                    # J_tmp = np.dot(self.Rbody,J_tmp)
                H,b = GenerateH(J_tmp,np.eye(3),0.5,np.array([1,1,1]))
                H_list.append(H)
                b_list.append(b)
        # 计算在单一方向上的冲量
        solution = CaculateFeasibleMargin(H_list,b_list,direction)
        if solution is None:
            print("WARNING: linprog has no solution!")
        else:
            print("direction is:",direction.reshape(3))
            print("solution is:",solution)


def limit_tau(f):
    '''
    f: one number
    '''
    max_f = 0.5
    if f > max_f:
        f = max_f
    elif f < -max_f:
        f = -max_f
    return f


def joint_pd_controller(qpos,qpos_tar,qvel):
    kp = 3.2
    kd = 0.015

    tau = kp*(qpos_tar - qpos) - kd * qvel
    # return limit_force(tau)
    return tau


model = load_model_from_path("../model/qingzhui2.xml")
sim = MjSim(model=model)
sim.model.opt.gravity[2] = -9.8

viewer = MjViewer(sim)
sim_state = sim.get_state()

qtar = np.array([0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543])
oqtar = np.array([0,0,0.5,1,0,0,0,0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543, 0, -0.72, 1.49543])
sim_state.qpos[:] = oqtar
print(sim_state)
sim.set_state(sim_state)

qz = qzrobot(sim)
qz.initialize()

while True:

    numscnt = 7200
    for i in range(numscnt):
        print('\niter',i)

        qz.Step()

        # 关节电机的控制增益G=200；输出力矩=控制输入*G
        print('torque:',sim.data.qfrc_actuator[6:])  # 各关节输出力矩
        print('ctrl:',sim.data.ctrl)                 # 各关节控制输入
        # print('actuator force:',sim.data.actuator_force)

        sim.step() 
        qz.ctime += 0.005  # 更新时间
        # import pdb
        # pdb.set_trace()
        viewer.render()

    break    
    if os.getenv('TESTING') is not None:
        break
