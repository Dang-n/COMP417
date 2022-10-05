'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''

import random
from threading import Timer
import time
import drawSample
import sys
import imageToRects
import utils

import math

def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )

# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append(p)
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    return [(p2[0]-p1[0]),(p2[1]-p1[1])]


def pointPointDistance(p1,p2):
    return math.sqrt(math.pow((p2[0]-p1[0]),2)+(math.pow((p2[1]-p1[1]),2)))


def closestPointToPoint(G,p2):
    closest = math.inf
    closestV = 0
    for v in G[nodes]:
        distance = pointPointDistance(vertices[v],p2)
        if distance < closest:
            closest = distance
            closestV = v

    return closestV


def lineHitsRect(p1,p2,r):

    #r[0] = x1, r[1] = y1, r[2] = x2, r[3] = y2 
    rTop = [r[0],r[1],r[2],r[1]]
    rBot = [r[0],r[3],r[2],r[3]]
    rLeft = [r[0],r[1],r[0],r[3]]
    rRight = [r[2],r[1],r[2],r[3]]
    sides = [rTop, rBot, rLeft, rRight]
    #Let p1 = (x3,y3) and p2 = (x4,y4)

    #s(x2-x1)-t(x4-x3)=x3-x1
    #s(y2-y1)-t(y4-y3)=y3-y1

    #Cramers rule: s = (x3-x1)(y4-y3)-(y3-y1)(x4-x3)/(x2-x1)(y4-y3)-(y2-y1)(x4-x3)
    #              t = (x2-x1)(y3-y1)-(y2-y1)(x3-x1)/(x2-x1)(y4-y3)-(y2-y1)(x4-x3)

    def det(A,B,C,D):
        return (A*C)-(B*D)

    for side in sides:
        paramS_num = det(p1[0]-side[0],p1[1]-side[1],p2[1]-p1[1],p2[0]-p1[0])
        paramS_den = det(side[2]-side[0],side[3]-side[1],p2[1]-p1[1],p2[0]-p1[0])
        if paramS_den == 0: return False
        
        paramT_num = det(side[2]-side[0],side[3]-side[1],p2[1]-side[1],p2[0]-side[0])
        paramT_den = det(side[2]-side[0],side[3]-side[1],p2[1]-p1[1],p2[0]-p1[0])
        if paramT_den == 0: return False
        paramS = paramS_num/paramS_den
        paramT = paramT_num/paramT_den
        if (paramS >= 0 and paramS <= 1) and  (paramT >= 0 and paramT <= 1):
            return True
        else:
            return False

def inRect(p,rect,dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """

    if p[0]<rect[0]-dilation: return 0
    if p[1]<rect[1]-dilation: return 0
    if p[0]>rect[2]+dilation: return 0
    if p[1]>rect[3]+dilation: return 0
    return 1


def addNewPoint(p1,p2,stepsize):
    #Add new point towards p2 from p1 at a distance stepsize away
    
    #Get vector from p1 to p2
    vec = lineFromPoints(p1,p2)
    vecLen = pointPointDistance(p1,p2)
    unitVec = [(vec[0]/vecLen),(vec[1]/vecLen)]

    newPoint=[p1[0]+(stepsize*unitVec[0]),p1[1]+(stepsize*unitVec[1])]
    return pointToVertex(newPoint)
    

def rrt_search(G, tx, ty, canvas):
    # Please carefully read the comments to get clues on where to start

    #Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    while 1: # Main loop
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        p = genvertex()
        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G,vertices[p])
        v = addNewPoint(vertices[cp],vertices[p],SMALLSTEP)

        if visualize:
            if nsteps%500 == 0: redraw(canvas)  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        redo = False
        for o in obstacles:
            # The following function defined by you must handle the occlusion cases

            if lineHitsRect(vertices[cp],vertices[v],o) or inRect(vertices[v],o,1):
                    vertices.pop(v)
                    redo = True
                    break

            #... reject
        
        if redo:
            continue

        G[nodes].append(v)
        G[edges].append( (cp, v) )
        if visualize:
            canvas.polyline( [vertices[cp], vertices[v]] )


        nsteps = nsteps+1
        if pointPointDistance(vertices[v], [tx,ty] ) < SMALLSTEP:
            print ("Target achieved.", nsteps, "nodes in entire tree")
            k = v
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((v, t))
                if visualize:
                    canvas.polyline([vertices[v], vertices[t]], 1)
                    while 1:
                        # backtrace and show the solution ...
                        k = returnParent(k, canvas)
                        canvas.events()
                        if k == 0: break  
                k = t
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print ("Path length", totaldist, "using", nsteps, "nodes.")

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print ("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print ("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break
        

def main():
    #seed
    
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')


    while 1:
        # graph G

        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )
        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':

    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
