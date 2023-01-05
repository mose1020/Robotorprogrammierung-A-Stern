# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
from math import cos, sin, pi
import numpy as np
import random


benchList = list()

# Benchmark 1
trapField = dict()
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionChecker(trapField), [[10,15]], [[10,1]], description, 2))


# Benchmark 2
bottleNeckField = dict()
bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
description = "Planer has to find a narrow passage."
benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField), [[4,15]], [[18,1]], description, 2))


# Benchmark 3
fatBottleNeckField = dict()
fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("Fat bottleneck", CollisionChecker(fatBottleNeckField), [[4,21]], [[18,1]], description, 2))


# Benchmark 4
simpleField = dict()
simpleField["obs1"] = Polygon([(0,5),(5,5),(5,4),(0,4)])
simpleField["obs2"] = Polygon([(10,5),(18,5),(18,4),(10,4)])
simpleField["obs3"] = Polygon([(21,5),(23,5),(23,4),(21,4)])
simpleField["obs4"] = Polygon([(10,23),(10,20),(11,20),(11,23)])
simpleField["obs5"] = Polygon([(10,16),(10,13),(11,13),(11,16)])
simpleField["obs6"] = Polygon([(11,13),(23,13),(23,14),(11,14)])
simpleField["obs7"] = Polygon([(0,21),(2,21),(2,23),(0,23)])
simpleField["obs8"] = Polygon([(0,17),(3.5,17),(3.5,15),(0,15)])
simpleField["obs9"] = Polygon([(0.5,7),(2.5,5.5),(3.5,6.5),(1.5,8)])
simpleField["obs10"] = Polygon([(0,10),(2,10),(2,12.5),(1,12.5),(1,11.25),(0,11.25)])
simpleField["obs11"] = Polygon([(6,23),(6,21.5),(7,21.5),(7,23)])
simpleField["obs12"] = Polygon([(18,23),(18,20),(20,20),(20,23)])
simpleField["obs13"] = Polygon([(11,14),(13,14),(13,17),(11,17)])
simpleField["obs14"] = Point(18,15.5).buffer(0.9)
simpleField["obs15"] = Polygon([(5,4),(8,3),(8,3.5),(5,4.5)])
simpleField["obs16"] = Polygon([(10,5),(7,6),(7,5.5),(10,4.5)])
simpleField["obs17"] = Polygon([(13.5,5),(16.5,5),(16.5,7.5),(13.5,7.5)])
simpleField["obs18"] = Polygon([(13+0.5,5+4),(17-0.5,5+4),(17-0.5,8+4),(13+0.5,8+4)])
simpleField["obs19"] = Polygon([(18,7),(22,7),(22,10),(18,10)])
simpleField["obs20"] = Polygon([(0,0.5),(7,0.5),(7,2),(0,2)])
description = "Find the path!"
benchList.append(Benchmark("B_rush",CollisionChecker(simpleField),[[1,17.7]],[[17.25,10]],description,2))


#Benchmark 5
Triangles= dict()
x=1
y=1.5
xoffset=2.5
yoffset=3
yoffset2=5.5
xoffsetcircle=5
#bottom1
Triangles["obs1"] = LineString([(x, y),           (x+xoffset, y+yoffset), (x+xoffset*2, y)]).buffer(0.07)
Triangles["obs2"] = LineString([(x+xoffset*2, y), (x+xoffset*3, y+yoffset), (x+xoffset*4, y)]).buffer(0.07)
Triangles["obs3"] = LineString([(x+xoffset*4, y), (x+xoffset*5, y+yoffset), (x+xoffset*6, y)]).buffer(0.07)
Triangles["obs4"] = LineString([(x+xoffset*6, y), (x+xoffset*7, y+yoffset), (x+xoffset*8, y)]).buffer(0.07)
#bottom2
Triangles["obs5"] = LineString([(x, y+yoffset2),           (x+xoffset, y+yoffset+yoffset2), (x+xoffset*2, y+yoffset2)]).buffer(0.07)
Triangles["obs6"] = LineString([(x+xoffset*2, y+yoffset2), (x+xoffset*3, y+yoffset+yoffset2), (x+xoffset*4, y+yoffset2)]).buffer(0.07)
Triangles["obs7"] = LineString([(x+xoffset*4, y+yoffset2), (x+xoffset*5, y+yoffset+yoffset2), (x+xoffset*6, y+yoffset2)]).buffer(0.07)
Triangles["obs8"] = LineString([(x+xoffset*6, y+yoffset2), (x+xoffset*7, y+yoffset+yoffset2), (x+xoffset*8, y+yoffset2)]).buffer(0.07)
#
Triangles["obs9"] = LineString([(x, y+yoffset2*2),           (x+xoffset, y+yoffset+yoffset2*2), (x+xoffset*2, y+yoffset2*2)]).buffer(0.07)
Triangles["obs10"] = LineString([(x+xoffset*2, y+yoffset2*2), (x+xoffset*3, y+yoffset+yoffset2*2), (x+xoffset*4, y+yoffset2*2)]).buffer(0.07)
Triangles["obs11"] = LineString([(x+xoffset*4, y+yoffset2*2), (x+xoffset*5, y+yoffset+yoffset2*2), (x+xoffset*6, y+yoffset2*2)]).buffer(0.07)
Triangles["obs12"] = LineString([(x+xoffset*6, y+yoffset2*2), (x+xoffset*7, y+yoffset+yoffset2*2), (x+xoffset*8, y+yoffset2*2)]).buffer(0.07)
#
Triangles["obs13"] = LineString([(x, y+yoffset2*3),           (x+xoffset, y+yoffset+yoffset2*3), (x+xoffset*2, y+yoffset2*3)]).buffer(0.07)
Triangles["obs14"] = LineString([(x+xoffset*2, y+yoffset2*3), (x+xoffset*3, y+yoffset+yoffset2*3), (x+xoffset*4, y+yoffset2*3)]).buffer(0.07)
Triangles["obs15"] = LineString([(x+xoffset*4, y+yoffset2*3), (x+xoffset*5, y+yoffset+yoffset2*3), (x+xoffset*6, y+yoffset2*3)]).buffer(0.07)
Triangles["obs16"] = LineString([(x+xoffset*6, y+yoffset2*3), (x+xoffset*7, y+yoffset+yoffset2*3), (x+xoffset*8, y+yoffset2*3)]).buffer(0.07)
#
Triangles["obs17"] = LineString([(x, y+yoffset2*3), (x, y+yoffset2*2)]).buffer(0.07)
Triangles["obs18"] = LineString([(x, y+yoffset2), (0, y+yoffset2)]).buffer(0.07)
Triangles["obs19"] = LineString([(x, y+yoffset2), (0, y+yoffset2)]).buffer(0.07)
Triangles["obs20"] = LineString([(x+xoffset*8, y+yoffset2*0), (x+xoffset*8, y+yoffset2*1)]).buffer(0.07)
Triangles["obs21"] = LineString([(x+xoffset*8, y+yoffset2*2), (25, y+yoffset2*2)]).buffer(0.07)
description = "Planer has to find a way through all the triangle."
benchList.append(Benchmark("Triangle fun", CollisionChecker(Triangles), [[18.5,7]], [[3.5,18]], description, 2))


#Benchmark 6
calc = dict()
draw = dict()
mid = list([10,10])
circle_size = 1
circle_dis = 3
circle_anz = 3
cut_with = 1

for i in range(circle_anz):
    calc["A"+str(i)] = Point(mid).buffer(2*circle_size+circle_dis*i)
    calc["B"+str(i)] = Point(mid).buffer(1*circle_size+circle_dis*i)
    a = random.randint(0,360)
    P_a = list([cos(a)*(circle_dis*circle_anz)+mid[0],sin(a)*(circle_dis*circle_anz)+mid[1]])
    calc["C"+str(i)] = LineString([(P_a),mid]).buffer(cut_with)
    calc["dif"+str(i)] = (calc["A"+str(i)].difference(calc["B"+str(i)])).difference(calc["C"+str(i)])
    draw["sol_1"+str(i)] = calc["dif"+str(i)]

description = "Find the path"
benchList.append(Benchmark("circle od death", CollisionChecker(draw), [mid],[[1,1]], description, 2))


# Benchmark 7
calc_W = dict()
draw_W = dict()
calc_D = dict()
draw_D = dict()
draw = dict()

Mainwallconers = list([(2,2),(18,2),(18,18),(2,18),(2,2)])
MainwallN = (Mainwallconers[(2)])[(1)]
MainwallO = (Mainwallconers[(2)])[(0)]
MainwallS = (Mainwallconers[(0)])[(1)]
MainwallW = (Mainwallconers[(0)])[(0)]

calc_W["Mainwall"] = LineString(Mainwallconers).buffer(0.2)
calc_W["Wall0"] = LineString([(10,MainwallS),(10,10),(MainwallW,10)]).buffer(0.1)
calc_W["Wall1"] = LineString([(10,10),(10,MainwallN)]).buffer(0.1)
calc_W["Wall2"] = LineString([(MainwallO,15),(10,15)]).buffer(0.1)
calc_W["Wall3"] = LineString([(12.5,15),(12.5,MainwallS)]).buffer(0.1)
calc_W["Wall4"] = LineString([(12.5,6),(MainwallO,6)]).buffer(0.1)
calc_W["Wall5"] = LineString([(12.5,11),(MainwallO,11)]).buffer(0.1)

calc_D["Door0"] = LineString([(11.3,MainwallS-0.5),(11.3,MainwallS+0.5)]).buffer(1)
calc_D["Door1"] = LineString([(9.5,8),(10.5,8)]).buffer(0.5)
calc_D["Door2"] = LineString([(9.5,11),(10.5,11)]).buffer(0.5)
calc_D["Door3"] = LineString([(11.5,14.5),(11.5,15.5)]).buffer(0.5)
calc_D["Door4"] = LineString([(12,8),(13,8)]).buffer(0.5)
calc_D["Door5"] = LineString([(12,4),(13,4)]).buffer(0.5)
calc_D["Door6"] = LineString([(14,15.5),(14,14.5)]).buffer(0.5)
calc_D["Door7"] = LineString([(16,10.5),(16,11.5)]).buffer(0.5)

for i in range(len(calc_W)-1):

    if i == 0:
        draw_W["House"+str(i)] = calc_W["Mainwall"].union(calc_W["Wall"+str(i)])
    elif i > 0:
        draw_W["House"+str(i)] = draw_W["House"+str(i-1)].union(calc_W["Wall"+str(i)])

for j in range(len(calc_D)):
    if j == 0:
        draw_D["House"+str(j)] = draw_W["House"+str(len(calc_W)-2)].difference(calc_D["Door"+str(j)])

    elif j > 0:
        draw_D["House"+str(j)] = draw_D["House"+str(j-1)].difference(calc_D["Door"+str(j)])
    
draw["1"]= draw_D["House"+str(len(calc_D)-1)]

description = "Find the path"
benchList.append(Benchmark("House of the rising sun", CollisionChecker(draw), [[15,7.5]],[[11.5,1]], description, 2))


#Benchmark 8
Stephan_2 = dict()
Stephan_2["obs1"] = LineString([(3, 3), (15,10), (20,3), (22,11), (20,20), (15,12), (3,19)]).buffer(0.5)
Stephan_2["obs2"] = LineString([(10,9), (2,11), (10,13)]).buffer(0.8)

description = "Planer has to find a way to the end"
benchList.append(Benchmark("Boring", CollisionChecker(Stephan_2), [[19,15]], [[22,17]], description, 2))


#Benchmark 9
person = dict()
person["arm_ol"] = LineString([(1.5,21.5),(9.4,13.5)]).buffer(0.8)
person["arm_or"] = LineString([(21.5,21.5),(13.6,13.5)]).buffer(0.8)
person["bein_ul"] = LineString([(1.5,1.5),(9.4,9)]).buffer(0.8)
person["bein_ur"] = LineString([(21.5,1.5),(13.6,9)]).buffer(0.8)
person["kopf"] = Point([11.5,18]).buffer(2.3)
person["körper"] = LineString([(11.5,8),(11.5,14)]).buffer(1)
person["haar1"] = LineString([(9.7,19.7),(9.3,20.5)]).buffer(0.2)
person["haar2"] = LineString([(10.5,20.3),(10.3,21.1)]).buffer(0.2)
person["haar3"] = LineString([(11.5,20.5),(11.5,21.3)]).buffer(0.2)
person["haar4"] = LineString([(12.5,20.3),(12.7,21.1)]).buffer(0.2)
person["haar5"] = LineString([(13.3,19.7),(13.7,20.5)]).buffer(0.2)
description = "Planer has to find the shortest way along the person."
benchList.append(Benchmark("Person", CollisionChecker(person), [[11.5,22]], [[11.5,1]], description, 2))


#Benchmark 10
trapField = dict()

#Rechtecke
trapField["obs1"] = Polygon([(5,5), (5, 10), (10, 10), (10, 5)])
trapField["obs2"] = Polygon([(5,15), (5, 20), (10, 20), (10, 15)])
trapField["obs3"] = Polygon([(15,5), (20, 5), (20, 10), (15, 10)])
trapField["obs4"] = Polygon([(15,15), (20, 15), (20, 20), (15, 20)])

#Beschreibung
description = "4 Rechtecke"
schwierigkeit = 1
benchList.append(Benchmark("4 Rechtecke", CollisionChecker(trapField), [[2,2]], [[21,21]], description, schwierigkeit))


#Benchmark 11
trapField = dict()
#Linker-Steg
trapField["obs1"] = Polygon([(5,0), (5, 5), (7, 5), (7, 0)])
trapField["obs2"] = Polygon([(5,15), (5, 8), (7, 8), (7, 15)])
trapField["obs3"] = Polygon([(5,25), (5, 18), (7, 18), (7, 25)])

#Mittel-Steg
trapField["obs4"] = Polygon([(12,0), (12, 1), (14, 1), (14, 0)])
trapField["obs5"] = Polygon([(12,22), (12, 3), (14, 3), (14, 22)])
trapField["obs6"] = Polygon([(12,25), (12, 24), (14, 24), (14, 25)])

#Rechter-Steg
trapField["obs8"] = Polygon([(19,0), (19, 5), (21, 5), (21, 0)])
trapField["obs9"] = Polygon([(19,15), (19, 8), (21, 8), (21, 15)])
trapField["obs10"] = Polygon([(19,25), (19, 18), (21, 18), (21, 25)])

#Beschreibung
description = "Gitter"
schwierigkeit = 3
benchList.append(Benchmark("Gitter", CollisionChecker(trapField), [[4,4]], [[24,24]], description, schwierigkeit))


#Benchmark 12
trapField = dict()
trapField["obs1"] = LineString([(1,0), (2, 20), (6, 8), (16, 8), (19,19)]).buffer(1)
trapField["obs2"] = LineString([(10,10), (6, 15), (25, 25), (20,7), (4, 2), (20,2), (15, 4)]).buffer(0.5)

#Beschreibung
description = "Labyrinth"
schwierigkeit = 4
benchList.append(Benchmark("Labyrinth", CollisionChecker(trapField), [[0.01,20]], [[14,4]], description, schwierigkeit))


#Benchmark 13
#TestEnv
labyrinth = dict()
labyrinth["topHorizontal"]      = LineString([(5, 15),      (20, 15)]).buffer(0.5)
labyrinth["bottomHorizontal"]   = LineString([(5, 5),       (20, 5)]).buffer(0.5)
labyrinth["safeTop"]            = LineString([(12, 15),   (12.5, 25)]).buffer(0.5)
labyrinth["safeTop"]            = LineString([(12, 5),    (12.5, 0)]).buffer(0.5)

for i in range(15):
    labyrinth['TopVert' + str(i)] = LineString([(5+i, 15), (5+i, 6)]).buffer(0.1)
    labyrinth['botomVert' + str(i)] = LineString([(5.5+i, 5), (5.5+i, 14)]).buffer(0.1)

description = "Labyrinth-Style Parkour"
benchList.append(Benchmark("FindMyWay", CollisionChecker(labyrinth), [[1, 10]], [[23, 10]], description, 2))