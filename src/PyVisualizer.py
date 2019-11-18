import sys, os, time
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import GLSimulationPlugin
from klampt.model.trajectory import Trajectory
from scipy.interpolate import interp1d
import ipdb
import copy
from scipy.spatial import ConvexHull
import draw_hull
from OpenGL.GL import *
import math
import numpy as np
import random

ExpName = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/"

# This function is used solely for the visualization of online contact planning.

class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self, world):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.quit = False
        self.starp = False

    def mousefunc(self, button, state, x, y):
        print("mouse",button,state,x,y)
        if button==2:
            if state==0:
                print("Click list...",[o.getName() for o in self.click_world(x,y)])
            return True
        return False

    def motionfunc(self, x, y, dx, dy):
        return False

    def keyboardfunc(self, c, x, y):
        print("Pressed",c)
        return True

    def click_world(self, x, y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s, d) = self.click_ray(x, y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit, pt) = g[1].rayCast(s, d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt, s), d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]

def my_draw_hull(h):
    glEnable(GL_LIGHTING)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1.0,0.25,0.5,0.5])
    draw_hull.draw_hull(h)

def String_List_to_Number_List(str_list, string_name):
    # This function is used to convert the string list to a certain type of list
    if string_name =="float":
        res_list = [float(i) for i in str_list]
    else:
        res_list = [int(i) for i in str_list]
    return res_list

def Configuration_Loader_fn(Config_Name):
    # This function is only used to load in the initial configuraiton
    # The initial file will be in the .config format
    with open(Config_Name,'r') as robot_angle_file:
        robotstate_angle_i = robot_angle_file.readlines()
    config_temp = [x.replace('\t',' ') for x in robotstate_angle_i]
    config_temp = [x.replace('\n','') for x in config_temp]
    config_temp = [float(i) for i in config_temp[0].split()]

    DOF = int(config_temp[0])
    # Config_Init = np.array(config_temp[1:])
    Config_Init = config_temp[1:]
    return DOF, Config_Init

def Read_Txt_fn(file_name):
    Empty_List = []
    with open(file_name) as Txt_File:
        Txt_File_Str = Txt_File.read().splitlines()
        for Txt_File_Str_i in Txt_File_Str:
            Txt_File_Str_i.replace("\'","")
            Empty_List.append(float(Txt_File_Str_i))
    return Empty_List

def State_Loader_fn(*args):
    if len(args) == 2:
        # In this case, the robot is only given the configuration file
        config_file_path = args[1] + args[0]
        DOF, Config_Init = Configuration_Loader_fn(config_file_path)
        # Then the Velocty_Init is set to be a zero value list
        Velocity_Init = []
        for i in range(0,DOF):
            Velocity_Init.append(0)
    else:
        if len(args) == 3:
            config_file_path = args[2] + args[0]
            velocity_file_path = args[2] + args[1]
            Config_Init = Read_Txt_fn(config_file_path)
            Velocity_Init = Read_Txt_fn(velocity_file_path)
            DOF = len(Config_Init)
        else:
            raise RuntimeError("Input name should be either one config file or two txt files!")
    return DOF, Config_Init, Velocity_Init

def Traj_Loader(robot_traj_path):
    with open(robot_traj_path,'r') as robot_traj_file:
        robot_traj_tot = robot_traj_file.readlines()
    return robot_traj_tot

def Traj_Loader_fn(*args):
    # This function is used to read in the traj file for visualization
    robot_traj_path = ""

    if len(args)==1:
        robot_traj_path = ExpName + "build/stateTraj" + args[0] + ".path"
    else:
        raise RuntimeError("Only one path file avaiable at one time!")

    # Load trajectories one by one
    robot_traj_tot = Traj_Loader(robot_traj_path)
    robot_traj = []
    time_count = 0;
    for robot_traj_i in robot_traj_tot:
        # Duration time
        delta_t_index = robot_traj_i.find('\t')
        if time_count ==0:
            delta_t = float(robot_traj_i[0:delta_t_index])
        time_count = 1
        robot_traj_i = robot_traj_i[delta_t_index + 1:]
        DOF_index = robot_traj_i.find('\t')
        DOF = int(robot_traj_i[0:DOF_index])
        robot_traj_i = robot_traj_i[DOF_index+1:]
        end_index = robot_traj_i.find('\n')
        robot_traj_i = robot_traj_i[0:end_index]
        robot_traj_i = robot_traj_i.split(" ")
        robot_traj_i = String_List_to_Number_List(robot_traj_i[0:DOF],"float")
        robot_traj.append(robot_traj_i)

    return delta_t, DOF, robot_traj

def Contact_Link_Reader(File_Name, Path_Name):
    # This function is used to read-in the formation of the certain link and its associated contact points
    # The output of this function is a dictionary of several keys with multiple list values
    # File_Name = "./User_File/Contact_Link.txt"
    Contact_Link_Dictionary = dict()
    # The format of this function should be an integet with a list of contact points
    Link_Number_i = -1
    File_Path_Name = Path_Name +File_Name
    with open(File_Path_Name) as Txt_File:
        Txt_File_Str = Txt_File.read().splitlines()
        Dictionary_Value_Add_Flag = 0
        for Txt_File_Str_i in Txt_File_Str:
            if "Link" in Txt_File_Str_i:
                # This indicates a contact link number
                # Then the next few points would be the local coordinates of the contact extremities
                Txt_File_Str_i = Txt_File_Str_i.translate(None, 'Link')     # This step is to get the link number out
                Link_Number_i = int(Txt_File_Str_i)
                Contact_Link_Dictionary[Link_Number_i] = []
                continue
            if Link_Number_i>=0:
                # Here the Txt_File_Str_i is a string of 'a, b, c' format
                Txt_File_Str_i = Txt_File_Str_i.split(",")
                del Txt_File_Str_i[-1]
                Txt_File_Flt_i = String_List_to_Number_List(Txt_File_Str_i,"float")
                Contact_Link_Dictionary[Link_Number_i].append(Txt_File_Flt_i)
    return Contact_Link_Dictionary

def Contact_Status_Reader(File_Name, Path_Name):
    # This function is used to read-in the formation of the certain link and its associated contact points
    # The output of this function is a dictionary of several keys with multiple list values
    # File_Name = "./User_File/Contact_Link.txt"
    Contact_Status_Dictionary = dict()
    # The format of this function should be an integet with a list of contact points
    Link_Number_i = -1
    File_Path_Name = Path_Name + File_Name
    with open(File_Path_Name) as Txt_File:
        Txt_File_Str = Txt_File.read().splitlines()
        Dictionary_Value_Add_Flag = 0
        for Txt_File_Str_i in Txt_File_Str:
            if "Link" in Txt_File_Str_i:
                # This indicates a contact link number
                # Then the next few points would be the local coordinates of the contact extremities
                Txt_File_Str_i = Txt_File_Str_i.translate(None, 'Link')     # This step is to get the link number out
                Link_Number_i = int(Txt_File_Str_i)
                Contact_Status_Dictionary[Link_Number_i] = []
                continue
            if Link_Number_i>=0:
                Contact_Status_Dictionary[Link_Number_i].append(int(Txt_File_Str_i))
    return Contact_Status_Dictionary

def Convex_Edge_Reader(File_Name, Path_Name):
    # This function is used to load in the Convex Edge for visualization
    # The format of this function should be an integer with a list of contact points
    File_Path_Name = Path_Name + File_Name
    Convex_Edge_List = []
    with open(File_Path_Name) as Txt_File:
        Txt_File_Str = Txt_File.read().splitlines()
        Dictionary_Value_Add_Flag = 0
        for Txt_File_Str_i in Txt_File_Str:
            Txt_File_Str_i = Txt_File_Str_i.split(" ")
            Txt_File_Flt_i = String_List_to_Number_List(Txt_File_Str_i,"float")
            Convex_Edge_List.append(Txt_File_Flt_i)
    return Convex_Edge_List

def PIP_Traj_Reader(index):

    PIPList = []
    EdgeAList = []
    EdgeBList = []
    EdgeCOMList = []
    EdgexList = []
    EdgeyList = []
    EdgezList = []

    EdgeA_path = ExpName + "build/EdgeATraj" + index + ".txt"
    with open(EdgeA_path,'r') as EdgeA_path_file:
        EdgeA_tot = EdgeA_path_file.readlines()
        for i in range(0, len(EdgeA_tot)):
            EdgeAList_i = []
            EdgeA_tot_i = EdgeA_tot[i].split(" ")
            EdgeAString = String_List_to_Number_List(EdgeA_tot_i[0:-1], "float")
            for j in range(0, len(EdgeAString)/3):
                EdgeAList_i.append(EdgeAString[3*j:3*j+3])
            EdgeAList.append(EdgeAList_i)

    EdgeB_path = ExpName + "build/EdgeBTraj" + index + ".txt"
    with open(EdgeB_path,'r') as EdgeB_path_file:
        EdgeB_tot = EdgeB_path_file.readlines()
        for i in range(0, len(EdgeB_tot)):
            EdgeBList_i = []
            EdgeB_tot_i = EdgeB_tot[i].split(" ")
            EdgeBString = String_List_to_Number_List(EdgeB_tot_i[0:-1], "float")
            for j in range(0, len(EdgeBString)/3):
                EdgeBList_i.append(EdgeBString[3*j:3*j+3])
            EdgeBList.append(EdgeBList_i)

    EdgeCOM_path = ExpName + "build/EdgeCOMTraj" + index + ".txt"
    with open(EdgeCOM_path,'r') as EdgeCOM_path_file:
        EdgeCOM_tot = EdgeCOM_path_file.readlines()
        for i in range(0, len(EdgeCOM_tot)):
            EdgeCOMList_i = []
            EdgeCOM_tot_i = EdgeCOM_tot[i].split(" ")
            EdgeCOMString = String_List_to_Number_List(EdgeCOM_tot_i[0:-1], "float")
            for j in range(0, len(EdgeCOMString)/3):
                EdgeCOMList_i.append(EdgeCOMString[3*j:3*j+3])
            EdgeCOMList.append(EdgeCOMList_i)

    Edgex_path = ExpName + "build/EdgexTraj" + index + ".txt"
    with open(Edgex_path,'r') as Edgex_path_file:
        Edgex_tot = Edgex_path_file.readlines()
        for i in range(0, len(Edgex_tot)):
            EdgexList_i = []
            Edgex_tot_i = Edgex_tot[i].split(" ")
            EdgexString = String_List_to_Number_List(Edgex_tot_i[0:-1], "float")
            for j in range(0, len(EdgexString)/3):
                EdgexList_i.append(EdgexString[3*j:3*j+3])
            EdgexList.append(EdgexList_i)

    Edgey_path = ExpName + "build/EdgeyTraj" + index + ".txt"
    with open(Edgey_path,'r') as Edgey_path_file:
        Edgey_tot = Edgey_path_file.readlines()
        for i in range(0, len(Edgey_tot)):
            EdgeyList_i = []
            Edgey_tot_i = Edgey_tot[i].split(" ")
            EdgeyString = String_List_to_Number_List(Edgey_tot_i[0:-1], "float")
            for j in range(0, len(EdgeyString)/3):
                EdgeyList_i.append(EdgeyString[3*j:3*j+3])
            EdgeyList.append(EdgeyList_i)

    Edgez_path = ExpName + "build/EdgezTraj" + index + ".txt"
    with open(Edgez_path,'r') as Edgez_path_file:
        Edgez_tot = Edgez_path_file.readlines()
        for i in range(0, len(Edgez_tot)):
            EdgezList_i = []
            Edgez_tot_i = Edgez_tot[i].split(" ")
            EdgezString = String_List_to_Number_List(Edgez_tot_i[0:-1], "float")
            for j in range(0, len(EdgezString)/3):
                EdgezList_i.append(EdgezString[3*j:3*j+3])
            EdgezList.append(EdgezList_i)
    PIPList.append(EdgeAList)
    PIPList.append(EdgeBList)
    PIPList.append(EdgeCOMList)
    PIPList.append(EdgexList)
    PIPList.append(EdgeyList)
    PIPList.append(EdgezList)
    return PIPList

def Convex_Edges_Plot(sim_robot, convex_edges_list, vis):
    Convex_Edges_Number = len(convex_edges_list)/2
    COM_Pos = sim_robot.getCom()
    for i in range(0, Convex_Edges_Number):
        EdgeA = convex_edges_list[2*i]
        EdgeB = convex_edges_list[2*i + 1]
        # Three edges to be added: A->B, A -> COM, B-> COM
        Edge_Index = str(i)
        vis.add("Edge:" + Edge_Index, Trajectory([0, 1], [EdgeA, EdgeB]))
        vis.hideLabel("Edge:" + Edge_Index, True)
        vis.setAttribute("Edge:" + Edge_Index,'width', 5.0)

def Robot_Config_Plot(world, DOF, state_ref, contact_link_dictionary, convex_edges_list, delta_t=0.5):
    # This function is used to plot the robot motion
    # The optimized solution is used to plot the robot motion and the contact forces

    # Initialize the robot motion viewer
    robot_viewer = MyGLPlugin(world)
    # Here it is to unpack the robot optimized solution into a certain sets of the lists

    vis.pushPlugin(robot_viewer)
    vis.add("world", world)
    vis.show()

    sim_robot = world.robot(0)
    sim_robot.setConfig(state_ref[0:DOF])

    contact_link_list = contact_link_dictionary.keys()

    while vis.shown():
        # This is the main plot program
        vis.lock()
        sim_robot.setConfig(state_ref[0:DOF])
        Convex_Edges_Plot(sim_robot, convex_edges_list, vis)
        Robot_COM_Plot(sim_robot, vis)
        vis.unlock()
        time.sleep(delta_t)

def PIP_Subplot(i, EdgeA, EdgeB, EdgeCOM, Edgex, Edgey, Edgez, COM_Pos, vis):
    scale = 0.25

    Edge_Index = str(i)
    vis.add("PIPEdge:" + Edge_Index, Trajectory([0, 1], [EdgeA, EdgeB]))
    vis.hideLabel("PIPEdge:" + Edge_Index, True)
    vis.setAttribute("PIPEdge:" + Edge_Index,'width', 7.5)

    vis.add("PIPEdgeCOM:" + Edge_Index, Trajectory([0, 1], [COM_Pos, EdgeCOM]))
    vis.hideLabel("PIPEdgeCOM:" + Edge_Index, True)
    vis.setAttribute("PIPEdgeCOM:" + Edge_Index,'width', 7.5)
    vis.setColor("PIPEdgeCOM:" + Edge_Index, 65.0/255.0, 199.0/255.0, 244.0/255.0, 1.0)

    # Local Coordinates
    Edgex_i = [ 0.0, 0.0, 0.0]
    Edgex_i[0] = EdgeCOM[0] + scale * Edgex[0]
    Edgex_i[1] = EdgeCOM[1] + scale * Edgex[1]
    Edgex_i[2] = EdgeCOM[2] + scale * Edgex[2]
    vis.add("PIPEdgex:" + Edge_Index, Trajectory([0, 1], [EdgeCOM, Edgex_i]))
    vis.hideLabel("PIPEdgex:" + Edge_Index, True)
    vis.setAttribute("PIPEdgex:" + Edge_Index,'width', 7.5)
    vis.setColor("PIPEdgex:" + Edge_Index, 1.0, 0.0, 0.0, 1.0)

    Edgey_i = [ 0.0, 0.0, 0.0]
    Edgey_i[0] = EdgeCOM[0] + scale * Edgey[0]
    Edgey_i[1] = EdgeCOM[1] + scale * Edgey[1]
    Edgey_i[2] = EdgeCOM[2] + scale * Edgey[2]
    vis.add("PIPEdgey:" + Edge_Index, Trajectory([0, 1], [EdgeCOM, Edgey_i]))
    vis.hideLabel("PIPEdgey:" + Edge_Index, True)
    vis.setAttribute("PIPEdgey:" + Edge_Index,'width', 7.5)
    vis.setColor("PIPEdgey:" + Edge_Index, 155.0/255.0, 244.0/255.0, 66.0/255.0, 1.0)

    Edgez_i = [ 0.0, 0.0, 0.0]
    Edgez_i[0] = EdgeCOM[0] + scale * Edgez[0]
    Edgez_i[1] = EdgeCOM[1] + scale * Edgez[1]
    Edgez_i[2] = EdgeCOM[2] + scale * Edgez[2]
    # print Edgez_i
    vis.add("PIPEdgez:" + Edge_Index, Trajectory([0, 1], [EdgeCOM, Edgez_i]))
    vis.hideLabel("PIPEdgez:" + Edge_Index, True)
    vis.setAttribute("PIPEdgez:" + Edge_Index,'width', 7.5)
    vis.setColor("PIPEdgez:" + Edge_Index, 68.0/255.0, 65.0/255.0, 244.0/255.0, 1.0)

def PIP_Remove(i, vis):
    # This function is used to remove the previous PIP to make sure that no residual PIPs exist.
    Edge_Index = str(i)
    vis.remove("PIPEdge:" + Edge_Index)
    vis.remove("PIPEdgeCOM:" + Edge_Index)
    vis.remove("PIPEdgex:" + Edge_Index)
    vis.remove("PIPEdgey:" + Edge_Index)
    vis.remove("PIPEdgez:" + Edge_Index)

def Robot_COM_Plot(sim_robot, vis):
    COMPos_start = sim_robot.getCom()
    COMPos_end = COMPos_start[:]
    COMPos_end[2] = COMPos_end[2] - 7.50
    vis.add("COM", Trajectory([0, 1], [COMPos_start, COMPos_end]))
    vis.hideLabel("COM",True)
    vis.setColor("COM", 0.0, 204.0/255.0, 0.0, 1.0)
    vis.setAttribute("COM",'width', 7.5)

def Reachable_Contact_Plot(vis, ReachableContacts_data):

    RowNo, ColumnNo = ReachableContacts_data.shape
    RowStart = 0
    RowEnd = RowNo
    # RowStart = 400000
    # RowEnd = 401000
    PointTol = 0.001;
    for i in range(RowStart, RowEnd):
        point_start = [0.0, 0.0, 0.0]
        point_end = [0.0, 0.0, 0.0]

        ReachableContact_i = ReachableContacts_data[i]
        point_start[0] = ReachableContact_i[0]
        point_start[1] = ReachableContact_i[1]
        point_start[2] = ReachableContact_i[2]

        vis.add("Point:" + str(i), point_start)
        vis.hideLabel("Point:" + str(i), True)
        vis.setColor("Point:" + str(i),65.0/255.0, 199.0/255.0, 244.0/255.0, 1.0)

        # point_end[0] = ReachableContact_i[0] + PointTol * random.uniform(0, 1)
        # point_end[1] = ReachableContact_i[1] + PointTol * random.uniform(0, 1)
        # point_end[2] = ReachableContact_i[2] + PointTol * random.uniform(0, 1)
        #
        # vis.add("Point:" + str(i), Trajectory([0, 1], [point_start, point_end]))
        # vis.hideLabel("Point:" + str(i), True)
        # vis.setAttribute("Point:" + str(i),'width', 7.5)

def Traj_Vis(world, DOF, robot_traj, PIP_traj, delta_t=0.5):
    # Initialize the robot motion viewer
    robot_viewer = MyGLPlugin(world)

    # Here it is to unpack the robot optimized solution into a certain sets of the lists
    vis.pushPlugin(robot_viewer)
    vis.add("world", world)
    vis.show()

    state_traj = robot_traj
    sim_robot = world.robot(0)
    EdgeAList = PIP_traj[0]
    EdgeBList = PIP_traj[1]
    EdgeCOMList = PIP_traj[2]
    EdgexList = PIP_traj[3]
    EdgeyList = PIP_traj[4]
    EdgezList = PIP_traj[5]

    TotalTrajLength = len(state_traj)
    EffectiveTrajLength = len(EdgeAList)
    RedundantTrajLength = TotalTrajLength - EffectiveTrajLength

    # Here we would like to read point cloud for visualization of planning.
    # Technically, there are three point clouds to be plotted.

    # 1. All Reachable Points
    IdealReachableContacts = ExpName + "build/IdealReachableContact.bin"
    f_IdealReachableContacts = open(IdealReachableContacts, 'rb')
    IdealReachableContacts_data = np.fromfile(f_IdealReachableContacts, dtype=np.double)
    IdealReachableContacts_data = IdealReachableContacts_data.reshape((IdealReachableContacts_data.size/3, 3))

    # 2. Active Reachable Points
    ActiveReachableContacts = ExpName + "build/ActiveReachableContact.bin"
    f_ActiveReachableContacts = open(ActiveReachableContacts, 'rb')
    ActiveReachableContacts_data = np.fromfile(f_ActiveReachableContacts, dtype=np.double)
    ActiveReachableContacts_data = ActiveReachableContacts_data.reshape((ActiveReachableContacts_data.size/3, 3))

    # 3. Contact Free Points
    ContactFreeContacts = ExpName + "build/ContactFreeContact.bin"
    f_ContactFreeContacts = open(ContactFreeContacts, 'rb')
    ContactFreeContacts_data = np.fromfile(f_ContactFreeContacts, dtype=np.double)
    ContactFreeContacts_data = ContactFreeContacts_data.reshape((ContactFreeContacts_data.size/3, 3))

    # 4. Supportive Points
    SupportContacts = ExpName + "build/SupportContact.bin"
    f_SupportContacts = open(SupportContacts, 'rb')
    SupportContacts_data = np.fromfile(f_SupportContacts, dtype=np.double)
    SupportContacts_data = SupportContacts_data.reshape((SupportContacts_data.size/3, 3))

    # # 3. Optimal Reachable Points
    # OptimalReachableContacts = ExpName + "build/OptimalReachableContact.bin"
    # f_OptimalReachableContacts = open(OptimalReachableContacts, 'rb')
    # OptimalReachableContacts_data = np.fromfile(f_OptimalReachableContacts, dtype=np.double)

    ReachablePoint = [0.0, 0.0, 0.0]

    while vis.shown():
        # This is the main plot program
        InfeasiFlag = 0
        for i in range(0, EffectiveTrajLength):
            vis.lock()
            config_i = state_traj[i + RedundantTrajLength]
            sim_robot.setConfig(config_i[0:DOF])
            COM_Pos = sim_robot.getCom()
            Robot_COM_Plot(sim_robot, vis)
            EdgeAList_i = EdgeAList[i]
            EdgeBList_i = EdgeBList[i]
            EdgeCOMList_i = EdgeCOMList[i]
            EdgexList_i = EdgexList[i]
            EdgeyList_i = EdgeyList[i]
            EdgezList_i = EdgezList[i]

            for j in range(0, len(EdgeAList_i)):
                EdgeA = EdgeAList_i[j]
                EdgeB = EdgeBList_i[j]
                EdgeCOM = EdgeCOMList_i[j]
                Edgex = EdgexList_i[j]
                Edgey = EdgeyList_i[j]
                Edgez = EdgezList_i[j]
                PIP_Subplot(j, EdgeA, EdgeB, EdgeCOM, Edgex, Edgey, Edgez, COM_Pos, vis)

            # if CPFlag is 1 or 2:
            #     try:
            #         h = ConvexHull(EdgeAList_i)
            #     except:
            #         InfeasiFlag = 1
            #     if InfeasiFlag is 0:
            #         h = ConvexHull(EdgeAList_i)
            #         hrender = draw_hull.PrettyHullRenderer(h)
            #         vis.add("blah", h)
            #         vis.setDrawFunc("blah", my_draw_hull)
            #     else:
            #         print "Input Contact Polytope Infeasible!"
            # Reachable_Contact_Plot(vis, IdealReachableContacts_data)
            # Reachable_Contact_Plot(vis, ActiveReachableContacts_data)
            Reachable_Contact_Plot(vis, ContactFreeContacts_data)
            # Reachable_Contact_Plot(vis, SupportContacts_data)
            vis.unlock()
            time.sleep(delta_t)

            for j in range(0, len(EdgeAList_i)):
                PIP_Remove(j, vis)

            # if((CPFlag is 1 or 2) and (InfeasiFlag is 0)):
            #     vis.remove("blah")



def main(*arg):
    Exp_Name = arg[0]
    Robot_Option = "../user/hrp2/"
    world = WorldModel()                    	# WorldModel is a pre-defined class

    # The next step is to load in robot's XML file
    XML_path = ExpName + "Envi1.xml"
    result = world.readFile(XML_path)         	# Here result is a boolean variable indicating the result of this loading operation
    if not result:
        raise RuntimeError("Unable to load model " + XML_path)
    Contact_Link_Dictionary = Contact_Link_Reader("ContactLink.txt", ExpName+"user/hrp2/")
    Contact_Status_Dictionary = Contact_Status_Reader("InitContact.txt", ExpName+"user/hrp2/")
    # ipdb.set_trace()
    if ".path" in Exp_Name:
        # Then this means that we are given the robot trajectories for experimentations.
        Exp_Name_Copy = copy.deepcopy(Exp_Name)
        Exp_File_Name = ""
        for str_i in Exp_Name_Copy:
            if str_i == ".":
                break;
            Exp_File_Name = Exp_File_Name + str_i;
        delta_t, DOF, robot_traj = Traj_Loader_fn(Exp_File_Name)
        PIPInfoList = PIP_Traj_Reader(Exp_File_Name)
        Traj_Vis(world, DOF, robot_traj, PIPInfoList, delta_t)

    else:
        # In this case, what we have is a config
        # The following function can be used in two ways: the first way is to load the Config_Init.config file while the second way is to load two
        DOF, Config_Init, Velocity_Init = State_Loader_fn(Exp_Name + ".config", Robot_Option)
        # According to the initial condition of the robot contact status, a basic optimization may have to be contacted to enforce the initial constraints.
        # Now it is the validation of the feasibility of the given initial condition
        State_Init = Config_Init + Velocity_Init
        Convex_Edges_List = Convex_Edge_Reader(Exp_Name + "CHEdges.txt", Robot_Option);
        Robot_Config_Plot(world, DOF, State_Init, Contact_Link_Dictionary, Convex_Edges_List)

if __name__ == "__main__":
    # main("InitConfig")
    main("1.path")
