import time
import random
import drawSample
import math
import sys
import imageToRects
import utils

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100 # 10 is good for normal real-time drawing

prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = args.step_size # what our "local planner" can handle.
map_size,obstacles = imageToRects.imageToRects(args.world)
#Note the obstacles are the two corner points of a rectangle
#Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [args.start_pos_x, args.start_pos_y, args.start_pos_theta], [args.start_pos_x, args.start_pos_y + args.robot_length, args.start_pos_theta] ]

# goal/target
tx = args.target_pos_x
ty = args.target_pos_y
tt = args.target_pos_theta
# start
sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
nodes=0
edges=1

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
        theta = random.random()*360
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
        theta = abs(random.gauss(tt, 360))
    else:
        print ("Not yet implemented")
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
            theta = random.random()*360
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
            theta = abs(random.gauss(tt, 360))
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if theta > 360: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y, theta]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
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

    newPoint=[p1[0]+(stepsize*unitVec[0]),p1[1]+(stepsize*unitVec[1]), p2[2]]
    return pointToVertex(newPoint)

class robot:
    length = args.robot_length
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def outOfBounds(self):
        if self.head()[0] > 0: return 0
        if self.head()[0] < XMAX: return 0
        if self.head()[1] > 0: return 0
        if self.head()[1] < YMAX: return 0
        return 1
         
    def tail(self):
        return [self.x, self.y]

    def head(self):
        return [self.x + (self.length*math.cos(math.radians(self.theta))), self.y + (self.length*math.sin(math.radians(self.theta)))]

    def changePose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def rrt_search(G, tx, ty, tt, canvas):
    #TODO
    #Fill this function as needed to work ...
    bot = robot(args.start_pos_x,args.start_pos_y,args.start_pos_theta)

    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    while 1:
        p = genvertex()
        cp = closestPointToPoint(G,vertices[p])
        v = addNewPoint(vertices[cp], vertices[p], SMALLSTEP)
        oldV = [bot.x, bot.y, bot.theta]
        bot.changePose(vertices[v][0], vertices[v][1], vertices[p][2])
        if bot.outOfBounds():
            bot.changePose(oldV[0], oldV[1], oldV[2])
            continue

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        redo = False
        for o in obstacles:
            #if inRect(p,o,1):
            if lineHitsRect(vertices[cp],vertices[v],o) or inRect(vertices[v],o,1):
                vertices.pop(v)
                bot.changePose(oldV[0], oldV[1], oldV[2])
                redo = True
                break
                #... reject
        if redo:
            continue

        for o in obstacles:
            if lineHitsRect(bot.tail(), bot.head(), o) or inRect(bot.tail(), o, 1) or inRect(bot.head(), o, 1):
                vertices.pop(v)
                bot.changePose(oldV[0], oldV[1], oldV[2])
                redo = True
                break
        if redo:
            continue

        G[nodes].append(v)
        G[edges].append( (cp,v) )
        if visualize:
            canvas.polyline(  [vertices[cp], vertices[v] ]  )

        nsteps = nsteps+1
        if pointPointDistance( vertices[v], [tx,ty] ) < SMALLSTEP:
            print ("Target achieved.", nsteps, "nodes in entire tree")
            k = v
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([bot.tail(), bot.head()], 1)
                    while 1:
                        # backtrace and show the solution ...
                        finish = False
                        for e in G[edges]:
                            if e[1]==k:
                                bot.changePose(vertices[e[1]][0], vertices[e[1]][1], vertices[e[1]][2])
                                canvas.polyline( [ bot.tail(), bot.head() ], style=3  )
                                k = e[0]
                            if k==0: 
                                finish = True
                                break
                        if finish: break
                k=t
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
                    print("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print("[" + d + "]")
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
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, tt, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    main()
