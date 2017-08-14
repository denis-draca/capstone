#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

string = ''


def callback(data):
    location = "/home/denis/catkin_ws/src/a_start/data/nodes.yaml"

    f = open(location, "w")
    global string
    node = 0

    first = True

    map = []

    for y in range(0, data.info.height):
        temp = []
        for x in range(y * data.info.width, y * data.info.width + data.info.width):
            temp.append(data.data[x])

        map.append(temp)

    node = 0
    node_list = []
    for y in range(0, len(map)):
        for x in range(0, len(map[y])):
            if map[y][x] > 0:
                node += 1
                continue

            f.write("node_" + str(node) + ":" + "\n")
            f.write(" x: " + str(x) + "\n")
            f.write(" y: " + str(y) + "\n")

            node_list.append(node)
            node += 1
            connected_nodes = []
            for y_temp in range(y-1, y+2):
                for x_temp in range(x-1, x+2):
                    if first:
                        print "y: " + str(y_temp) + " x: " + str(x_temp) + " cx:" + str(x) + " Cy:" + str(y)
                    if y_temp == y and x_temp == x:
                        continue

                    if map[y_temp][x_temp] < 100:
                        connected_nodes.append(y_temp*data.info.width + x_temp)

            f.write(" connected_nodes: [")
            for temp_nodes in connected_nodes[:-1]:
                f.write(str(temp_nodes) + ",")

            if len(connected_nodes) > 0:
                f.write(str(connected_nodes[len(connected_nodes)-1]) + "]" + "\n")
            else:
                f.write("]\n")

            first = False

    # f.write("total_nodes: " + str(node) + "\n")
    f.write(" node_list: [")
    for temp_nodes in node_list[:-1]:
        f.write(str(temp_nodes) + ",")

    if len(connected_nodes) > 0:
        f.write(str(node_list[len(node_list) - 1]) + "]" + "\n")
    else:
        f.write("]\n")
    f.close()
    rospy.signal_shutdown('node_maker')


rospy.init_node('node_maker', anonymous=True)
rospy.Subscriber("/map", OccupancyGrid, callback)

rospy.spin()
