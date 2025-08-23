#!/usr/bin/env python

""" 
利用 dynamic_corridor.py 尝试构建走廊地图
"""
import random
import roslib
import rospy
import math
from mader_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

import numpy as np
from numpy import linalg as LA
import random

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pyquaternion import Quaternion
import tf

from math import sin, cos, tan


import copy 
import sys

# 动态、静态颜色区分（红/绿/蓝/不透明度）
color_static=ColorRGBA(r=0,g=0,b=1,a=1);
color_dynamic=ColorRGBA(r=1,g=0,b=0,a=1);

# 走廊地图类，走廊可以理解成无人机的活动范围，而不是地图尺寸
""" total_num_obs --- 总障碍物数
    num_of_dyn_objects --- 动态障碍物数量/占65% /系统设定
    num_of_stat_objects --- 静态障碍物数量
    self.x_min
    self.x_max --- x方向范围
    self.y_min
    self.y_max --- y方向范围    
    self.z_min
    self.z_max --- z方向范围
    self.scale --- 缩放因子           scale是一个缩放因子(默认 1.0)，用于控制障碍物可以超出走廊边界的距离
    self.slower_min --- 最小减速因子
    self.slower_max --- 最大减速因子   ????
    self.bbox_dynamic --- 动态障碍物尺寸（长、宽、高一样）  
    self.bbox_static_vert --- 垂直静态障碍物尺寸（细高）
    self.bbox_static_horiz --- 水平静态障碍物尺寸（长而扁）
    self.percentage_vert --- 垂直障碍物所占比例  在后面初始化时有用
"""
class MovingCorridor:
    def __init__(self, total_num_obs):
        print(total_num_obs)
        self.total_num_obs=total_num_obs;
        self.num_of_dyn_objects=int(0.65*total_num_obs);
        self.num_of_stat_objects=total_num_obs-self.num_of_dyn_objects; #They are actually dynamic obstacles
        self.x_min= 2.0
        self.x_max= 75.0
        self.y_min= -3.0 
        self.y_max= 3.0
        self.z_min = 1.0 
        self.z_max = 2.0
        self.scale =1.0;
        self.slower_min =1.1
        self.slower_max = 1.1
        self.bbox_dynamic =[0.8, 0.8, 0.8] #[0.6, 0.6, 0.6]
        self.bbox_static_vert = [0.4, 0.4, 4]
        self.bbox_static_horiz = [0.4, 8, 0.4]
        self.percentage_vert = 0.35;

# 仿真环境包装器
class FakeSim:
    def __init__(self, total_num_obs):
        # state 负责存储仿真时的信息
        self.state = State()          
        # 获取命名空间   
        name = rospy.get_namespace()
        # 去/ 得到干净的命名空间
        self.name = name[1:-1]
        # 创建走廊式世界
        world_type="MovingCorridor"
        self.world = MovingCorridor(total_num_obs)

        # 定义静态和动态障碍物使用的3D模型资源路径
        available_meshes_static=["package://mader/meshes/ConcreteDamage01b/model3.dae", "package://mader/meshes/ConcreteDamage01b/model2.dae"]
        available_meshes_dynamic=["package://mader/meshes/ConcreteDamage01b/model4.dae"]
        
        # 共同定义所有障碍物的三维位置 x y z offset(偏移量)
        self.x_all=[];
        self.y_all=[];
        self.z_all=[];
        self.offset_all=[];
        # slower控制动态障碍物的移动速度 ???
        self.slower=[];
        # meshes定义障碍物的可视化外观
        self.meshes=[];
        # type区分静态障碍物（固定位置）和动态障碍物（可移动）
        self.type=[];#"dynamic" or "static_vert" or "static_horiz"
        # bboxes定义轴对齐的碰撞框（AABB），用于快速碰撞检测
        self.bboxes=[]; 

        # 初始化动态障碍物
        for _ in range(self.world.num_of_dyn_objects):          
            self.x_all.append(random.uniform(self.world.x_min, self.world.x_max));
            self.y_all.append(random.uniform(self.world.y_min, self.world.y_max));
            self.z_all.append(random.uniform(self.world.z_min, self.world.z_max));
            self.offset_all.append(random.uniform(-2*math.pi, 2*math.pi));  # ？？？？什么意思
            self.slower.append(random.uniform(self.world.slower_min, self.world.slower_max));
            self.type.append("dynamic")
            self.meshes.append(random.choice(available_meshes_dynamic));
            self.bboxes.append(self.world.bbox_dynamic);
        
        # 初始化静态障碍物，由于垂直和水平障碍物在z轴有别，先做处理
        for i in range(self.world.num_of_stat_objects):
            bbox_i=[]; 
            if(i<self.world.percentage_vert*self.world.num_of_stat_objects):
                bbox_i=self.world.bbox_static_vert;
                self.z_all.append(bbox_i[2]/2.0);
                self.type.append("static_vert")
            else:
                bbox_i=self.world.bbox_static_horiz;
                self.z_all.append(random.uniform(0.0, 3.0));
                self.type.append("static_horiz")
            
            self.x_all.append(random.uniform(self.world.x_min-self.world.scale, self.world.x_max+self.world.scale));
            self.y_all.append(random.uniform(self.world.y_min-self.world.scale, self.world.y_max+self.world.scale));
            
            self.offset_all.append(random.uniform(-2*math.pi, 2*math.pi));
            self.slower.append(random.uniform(self.world.slower_min, self.world.slower_max));

            self.meshes.append(random.choice(available_meshes_static));
            self.bboxes.append(bbox_i)

        # 发布动态轨迹（DynTraj类型）消息
        # queue_size​​: 设置为障碍物总数 (total_num_obs)，确保能处理高频率的轨迹更新
        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=self.world.total_num_obs)#If queue_size=1, pubTraj will not be able to keep up (due to the loop in pubTF)#, latch=True
        # 发布 Marker消息，用于在 RVIZ 中可视化 ​​静态障碍物的几何形状​​（如立方体、圆柱体）
        self.pubShapes_static = rospy.Publisher('/shapes_static', Marker, queue_size=1, latch=True)
        # 发布 MarkerArray消息，用于在 RVIZ 中显示 ​​带3D网格模型的静态障碍物​​（如 .dae或 .stl文件）
        self.pubShapes_static_mesh = rospy.Publisher('/shapes_static_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/shapes_dynamic_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic = rospy.Publisher('/shapes_dynamic', Marker, queue_size=1, latch=True)
        # 向 Gazebo 仿真器发送指令，直接控制模型的位姿（位置、姿态、速度等）
        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        
        # 静态障碍物发布标志位
        self.already_published_static_shapes=False;
        rospy.sleep(0.5) # 延时确保上述操作

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()

        # 障碍物消息基础模板
        marker_tmp=Marker();
        marker_tmp.header.frame_id="world"
        marker_tmp.type=marker_tmp.CUBE_LIST;
        marker_tmp.action=marker_tmp.ADD;

        marker_static=copy.deepcopy(marker_tmp);
        marker_dynamic=copy.deepcopy(marker_tmp);

        marker_dynamic.color=color_dynamic;
        marker_static.color=color_static;

        ###################3
        marker_array_static_mesh=MarkerArray();
        marker_array_dynamic_mesh=MarkerArray();

        for i in range(self.world.num_of_dyn_objects + self.world.num_of_stat_objects):
            t_ros=rospy.Time.now()
            t=rospy.get_time(); #Same as before, but it's float

            dynamic_trajectory_msg = DynTraj(); 

            bbox_i=self.bboxes[i];
            s=self.world.scale;
            if(self.type[i]=="dynamic"):

              [x_string, y_string, z_string] = self.trefoil(self.x_all[i], self.y_all[i], self.z_all[i], s,s,s, self.offset_all[i], self.slower[i]) 
              # print("self.bboxes[i]= ", self.bboxes[i])
              dynamic_trajectory_msg.bbox = bbox_i;
              marker_dynamic.scale.x=bbox_i[0]
              marker_dynamic.scale.y=bbox_i[1]
              marker_dynamic.scale.z=bbox_i[2]
            else:
              # [x_string, y_string, z_string] = self.static(self.x_all[i], self.y_all[i], self.z_all[i]);
              dynamic_trajectory_msg.bbox = bbox_i;
              marker_static.scale.x=bbox_i[0]
              marker_static.scale.y=bbox_i[1]
              marker_static.scale.z=bbox_i[2]
              if(self.type[i]=="static_vert"):
                [x_string, y_string, z_string] = self.wave_in_z(self.x_all[i], self.y_all[i], self.z_all[i], s, self.offset_all[i], 1.0)
              else:
                [x_string, y_string, z_string] = self.wave_in_z(self.x_all[i], self.y_all[i], self.z_all[i], s, self.offset_all[i], 1.0)

            # eval()会解析字符串中的变量和运算符，最终返回数值结果（整数或浮点数）里面可能有t，也就是说，会得到初始位姿            
            x = eval(x_string)
            y = eval(y_string)
            z = eval(z_string)

            dynamic_trajectory_msg.is_agent=False;
            dynamic_trajectory_msg.header.stamp= t_ros;
            dynamic_trajectory_msg.function = [x_string, y_string, z_string]
            dynamic_trajectory_msg.pos.x=x #Current position
            dynamic_trajectory_msg.pos.y=y #Current position
            dynamic_trajectory_msg.pos.z=z #Current position

            dynamic_trajectory_msg.id = 4000+ i #Current id 4000 to avoid interference with ids from agents 

            self.pubTraj.publish(dynamic_trajectory_msg)
            # 发布变换关系, 就像注册了一个服务，之后随时都可以调用这个服务来获取坐标变换信息
            br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name+str(dynamic_trajectory_msg.id), "world")
            
            # If you want to see the objects in rviz 障碍物的可视化处理，前面那个只是创建
            point=Point()
            point.x=x;
            point.y=y;
            point.z=z;

            ##########################
            marker=Marker();
            marker.id=i;
            marker.ns="mesh";
            marker.header.frame_id="world"
            marker.type=marker.MESH_RESOURCE;
            marker.action=marker.ADD;

            marker.pose.position.x=x
            marker.pose.position.y=y
            marker.pose.position.z=z
            marker.pose.orientation.x=0.0;
            marker.pose.orientation.y=0.0;
            marker.pose.orientation.z=0.0;
            marker.pose.orientation.w=1.0;
            # ​lifetime=0.0​​ 表示该Marker ​​永久存在​​，直到被显式删除或覆盖
            marker.lifetime = rospy.Duration.from_sec(0.0);
            marker.mesh_use_embedded_materials=True
            marker.mesh_resource=self.meshes[i]

            if(self.type[i]=="dynamic"):
                marker_dynamic.points.append(point);

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];

                marker_array_dynamic_mesh.markers.append(marker);

            if(self.type[i]=="static_vert" or self.type[i]=="static_horiz"):
                marker_static.points.append(point);

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];
                
                marker_array_static_mesh.markers.append(marker);
        
        # 发布动态障碍物材质+物体
        self.pubShapes_dynamic_mesh.publish(marker_array_dynamic_mesh)
        self.pubShapes_dynamic.publish(marker_dynamic)
        
        # 发布静态障碍物材质+物体
        self.pubShapes_static_mesh.publish(marker_array_static_mesh)
        self.pubShapes_static.publish(marker_static)

    def static(self,x,y,z):
        return [str(x), str(y), str(z)]
    
    # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset, slower):

        # offset slower 实现了 sin(at+b) 的类似功能
        # slower=1.0; #The higher, the slower the obstacles move" 
        tt='t/' + str(slower)+'+';

        x_string=str(scale_x)+'*(sin('+tt +str(offset)+') + 2 * sin(2 * '+tt +str(offset)+'))' +'+' + str(x); #'2*sin(t)' 
        y_string=str(scale_y)+'*(cos('+tt +str(offset)+') - 2 * cos(2 * '+tt +str(offset)+'))' +'+' + str(y); #'2*cos(t)' 
        z_string=str(scale_z)+'*(-sin(3 * '+tt +str(offset)+'))' + '+' + str(z);                               #'1.0'        

        # x_string='sin('+tt +str(offset)+')';
        # y_string='cos('+tt +str(offset)+')';
        # z_string='1'

        return [x_string, y_string, z_string]
    
    def wave_in_z(self,x,y,z,scale, offset, slower):

        tt='t/' + str(slower)+'+';

        x_string=str(x);
        y_string=str(y)
        z_string=str(scale)+'*(-sin( '+tt +str(offset)+'))' + '+' + str(z);                     

        return [x_string, y_string, z_string]

    
# 创建仿真环境对象 FakeSim。
# 启动一个高频定时器（100Hz），周期性调用 pubTF方法更新障碍物状态。
# 进入ROS主循环（rospy.spin()），保持节点运行。
def startNode(total_num_obs):
    c = FakeSim(total_num_obs)
    rospy.Timer(rospy.Duration(0.01), c.pubTF)
    rospy.spin()

# 主函数对 argv(障碍物个数) 做一些处理
if __name__ == '__main__':

    # TODO: Use instead https://docs.python.org/3.3/library/argparse.html
    print("********************************")
    print(sys.argv)
    if(len(sys.argv)<=1):
        total_num_obs=140; 
    else:
        total_num_obs=int(sys.argv[1])

    ns = rospy.get_namespace()
    try:
        rospy.init_node('my_map_test01')
        startNode(total_num_obs)
    except rospy.ROSInterruptException:
        pass