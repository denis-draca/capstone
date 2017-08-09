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
    f.write("total_nodes: " + str(data.info.height * data.info.width) + "\n")

    map = []

    for y in range(0, data.info.height):
        current_x = 0;
        temp = []
        for x in range(y * data.info.width, y * data.info.width + data.info.width):
            temp.append(data.data[x])
            # node += 1
            #
            # if data.data[x] > 0:
            #     continue
            #
            # f.write("node_" + str(node) + ":" + "\n")
            # f.write(" x: " + str(current_x) + "\n")
            # f.write(" y: " + str(y) + "\n")
            #
            # connected_nodes = []
            #
            # read_pos1 = x - 1
            # read_pos2 = x + 1
            # read_pos3 = x - data.info.width - 1
            # read_pos4 = x - data.info.width + 1
            # read_pos5 = x - data.info.width
            # read_pos6 = x + data.info.width -1
            # read_pos7 = x + data.info.width + 1
            # read_pos8 = x + data.info.width
            #
            # if read_pos1 < y * data.info.width:
            #     print "1"
            # else:
            #     if data.data[read_pos1] < 100:
            #         connected_nodes.append(read_pos1)
            #
            # if read_pos2 > y * data.info.width + data.info.width:
            #     print "2"
            # else:
            #     if data.data[read_pos2] < 100:
            #         connected_nodes.append(read_pos2)
            #
            # if y - 1 < 0:
            #     print "3"
            #
            # else:
            #     if read_pos3 < (y -1) * data.info.width:
            #         print "4"
            #     else:
            #         if data.data[read_pos3] < 100:
            #             connected_nodes.append(read_pos3)
            #
            # if y - 1 < 0:
            #     print "5"
            # else:
            #     if read_pos4 > (y - 1) * data.info.width + data.info.width:
            #         print "6"
            #     else:
            #         if data.data[read_pos4] < 100:
            #             connected_nodes.append(read_pos4)
            #
            # if y - 1 < 0:
            #     print "7"
            # else:
            #     if data.data[read_pos5] < 100:
            #         connected_nodes.append(read_pos5)
            #
            # if y + 1 > data.info.height:
            #     print "8"
            # else:
            #     if read_pos6 < (y + 1) * data.info.width:
            #         print "9"
            #     else:
            #         if data.data[read_pos6] < 100:
            #             connected_nodes.append(read_pos6)
            #
            # if y + 1 > data.info.height:
            #     print "10"
            # else:
            #     if read_pos7 > (y + 1) * data.info.width + data.info.width:
            #         print "11"
            #     else:
            #         if data.data[read_pos7] < 100:
            #             connected_nodes.append(read_pos7)
            #
            # if y + 1 > data.info.height:
            #     print "12"
            # else:
            #     if data.data[read_pos8] < 100:
            #         connected_nodes.append(read_pos8)
            #
            #
            # first = False
            # f.write(" connected_nodes: [")
            # for temp_nodes in connected_nodes[:-1]:
            #     f.write(str(temp_nodes) + ",")
            #
            # if len(connected_nodes) > 0:
            #     f.write(str(connected_nodes[len(connected_nodes)-1]) + "]" + "\n")
            # else:
            #     f.write("]\n")
            #
            # current_x += 1

        map.append(temp)

    node = 0
    for y in range(0, len(map)):
        for x in range(0, len(map[y])):
            if map[y][x] > 0:
                node += 1
                continue

            f.write("node_" + str(node) + ":" + "\n")
            f.write(" x: " + str(x) + "\n")
            f.write(" y: " + str(y) + "\n")

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

    f.close()
    rospy.signal_shutdown('node_maker')


rospy.init_node('node_maker', anonymous=True)
rospy.Subscriber("/map", OccupancyGrid, callback)

rospy.spin()
