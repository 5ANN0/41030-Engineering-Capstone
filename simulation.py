import matplotlib.pyplot as plt
import math
import numpy as np
import random

figure, axes = plt.subplots()

def genrate_xy_coodrinates(range_min,range_max):

    ## Generate random coordinates for the robot and landmarks

    x_pos = random.uniform(range_min,range_max)
    y_pos = random.uniform(range_min,range_max)

    return (x_pos,y_pos)

def point(x,y,sym):

    ## Plot the robot and landmarks on the figure

    plt.plot(x, y, sym, ms=5)

    return x,y

def robot_orientation(robot_pose,angle=0,test=False):

    ## set the orientation of the robot

    x,y = robot_pose

    ## randomly generated orinetation
    if test == False:
        deg_angle = random.randint(-180,180)
        angle = math.radians(deg_angle)
        #print (deg_angle, " calclulated ", math.degrees(angle))

    
    distance = 100 ## draw the positive x-axis of the robot

    x2 = x + distance*math.cos(angle)
    # print(x,x2)
    y2 = y + distance*math.sin(angle)
    # (print(y, y2))
    plt.plot([x, x2], [y, y2],':')

    return angle

  



def sensor_sim(robot_pos,landmark_pos,robot_orient):

    ## simulate the sensor by returning the brearing and range

    #print ("rob_orient ", math.degrees(robot_orient))

    range = math.dist(robot_pos,landmark_pos)
    #print("range = ",range)

    #print(robot_pos[0], landmark_pos[0], robot_pos[1], landmark_pos[1])

            #x1,              x2                      y1           y2
    ## plot the distance to the landmark from the robot

    ##plot dotted lines from landmarks to robot
    plt.plot([robot_pos[0], landmark_pos[0]], [robot_pos[1], landmark_pos[1]],':')

    ## draw cirscles with each landmark as the center to visualise the trilateration method
    circle = plt.Circle(landmark_pos,range,fill=False)
    axes.add_artist(circle)

    ## find the angle relative to the robot
    if (landmark_pos[0]-robot_pos[0]) != 0:

        raw_angle = np.atan2((landmark_pos[1]-robot_pos[1]),(landmark_pos[0]-robot_pos[0]))
        #print("Raw angle ",math.degrees(raw_angle))
        angle = (raw_angle-(robot_orient)) 
    else:
        angle = math.radians(90)-robot_orient
    #print ("Angle = ", math.degrees(angle))

    return (range,angle)

def trilateration_method(satellite_pos,range, angle):
    ## known variable
    #X-axis of the satelites
    x_a = satellite_pos[0,0]
    x_b = satellite_pos[1,0]
    x_c = satellite_pos[2,0]

    #Y-axis of the satelites

    y_a = satellite_pos[0,1]
    y_b =satellite_pos[1,1]
    y_c =satellite_pos[2,1]

    # range from the robot to each satellite
    range_a = range[0]
    range_b = range[1]
    range_c = range [2]

    #bearing of each satelites form the robot 
    angle_a = angle[0]
    angle_b = angle[1]
    angle_c = angle[2]
    
    ## trilateration method from the equations of 3 circles 
    ## p = m1 X m2 from https://www.mdpi.com/1424-8220/22/16/6085#Simulation_Evaluation

    m1 = np.array([[2*(x_a-x_c),2*(y_a-y_c)],
                  [2*(x_b-x_c),2*(y_b-y_c)]])
    #print (m1)

    m2 = np.array([x_a**2 - x_c**2 + y_a**2 - y_c**2 + range_c**2 - range_a**2,
                   x_b**2 - x_c**2 + y_b**2 - y_c**2 + range_c**2 - range_b**2])
    #print (m2)
   
    robot_xy = np.linalg.solve(m1,m2)

    print ("Calulated x and y cordenates of the Robot : ",robot_xy)

    #print ("y_a = ", y_a, "x_a = ", x_a, " x = ", robot_xy[0]," y = ", robot_xy[1])

    ## calulate the orinetation of the robot

    #using landmark A
    raw_angle_a = np.atan2(y_a-robot_xy[1],x_a-robot_xy[0])

    #print(np.degrees(raw_angle_a))
                         
    offset_angle = raw_angle_a-angle_a

    print("Calculated orientation = ", np.degrees(offset_angle))







## plot_point((10,20),30,20)

def main():
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    robot_pos = genrate_xy_coodrinates(0,10)
    print ("Simulated Robot coodrinates ", robot_pos)
    r_x,r_y = point(robot_pos[0],robot_pos[1],'H')
    robot_orient = robot_orientation(robot_pos)

    print ("Simulated Robot orientation ", math.degrees(robot_orient))

    landmark_pos_a = genrate_xy_coodrinates(0,10)
    landmark_pos_b = genrate_xy_coodrinates(0,10)
    landmark_pos_c = genrate_xy_coodrinates(0,10)


    la_X,la_y = point(landmark_pos_a[0],landmark_pos_a[1],'o')
    axes.annotate("La",(la_X,la_y))
    lb_X,lb_y = point(landmark_pos_b[0],landmark_pos_b[1],'o')
    axes.annotate("Lb",(lb_X,lb_y))
    lc_X,lc_y = point(landmark_pos_c[0],landmark_pos_c[1],'o')
    axes.annotate("Lc",(lc_X,lc_y))



    r1,a1 = sensor_sim((r_x,r_y),(la_X,la_y),robot_orient)
    r2,a2 = sensor_sim((r_x,r_y),(lb_X,lb_y),robot_orient)
    r3,a3 = sensor_sim((r_x,r_y),(lc_X,lc_y),robot_orient)

    landmrks_xy = (np.array([[la_X,la_y],[lb_X,lb_y],[lc_X,lc_y]]))
    landmrks_range = np.array([r1,r2,r3])
    landmrks_angle = np.array([a1,a2,a3])
    trilateration_method(landmrks_xy,landmrks_range,landmrks_angle)
    
    
    plt.show()
    




### TEST
def test():
    
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    offset_angle = math.radians(45)

    r_x,r_y  = point(5,3,'H')
    robot_orient = robot_orientation((r_x,r_y),offset_angle,True)

    la_X,la_y = point(5,6,'o')
    lb_X,lb_y = point(7,8,'o')
    lc_X,lc_y = point(-9,10,'o')

    r1,a1 = sensor_sim((r_x,r_y),(la_X,la_y),robot_orient)
    r2,a2 = sensor_sim((r_x,r_y),(lb_X,lb_y),robot_orient)
    r3,a3 = sensor_sim((r_x,r_y),(lc_X,lc_y),robot_orient)

    landmrks_xy = (np.array([[la_X,la_y],[lb_X,lb_y],[lc_X,lc_y]]))
    landmrks_range = np.array([r1,r2,r3])
    landmrks_angle = np.array([a1,a2,a3])
    trilateration_method(landmrks_xy,landmrks_range,landmrks_angle)
   
    
    plt.show()






if __name__ == "__main__":
    main()
