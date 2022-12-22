from controller import Robot, Supervisor, Emitter
import random, json
import math
from math import sqrt, acos, asin

MAX_COLLECTIBLES = 8
RUPEE_REDUCTION = 5
RATE_CHANGE_TIME = 60000

positions = set()
positions.add((-1.89, 1.89))
positions.add((-0.93328, 1.9303))
positions.add((-0.0890697, 1.10696))
positions.add((0.600138, 0.948865))
positions.add((1.67731, 1.10951))
positions.add((1.0773, 0.176304))
positions.add((0.0888764, -0.242643))
positions.add((-0.264078, -0.26325))
positions.add((-0.902467, 0.261876))
positions.add((-1.4484, -0.384811))
positions.add((-1.41139, -1.26089))
positions.add((-1.92742, -1.91008))
positions.add((-0.975598, -1.68874))
positions.add((0.183692, -1.7457))
positions.add((0.435744, -0.773298))
positions.add((1.23528, -1.2919))
positions.add((1.91607, -1.91091))
positions.add((-1.4676, 0.764517))
positions.add((-0.8876, 0.914517))
positions.add((-0.0976, 0.314517))
positions.add((0.6724, 1.62452))
positions.add((1.4124, -0.61548))
positions.add((-0.5476, -1.07548))

supervisor = Supervisor()
root_node = supervisor.getRoot()
children = root_node.getField("children")
timestep = int(supervisor.getBasicTimeStep())
collectibles = []
robot = supervisor.getFromDef('dave')
emitter = supervisor.getDevice("emitter")

rupees = 0
dollars = 0
currentTime = 0
gameTime = 0
goalRates = [300, 300, 300, 300]
goalPositions = [(-2.11668, -0.00301332, 0.0), (0.00688451, 2.10708, 0.0), (2.10789, -0.00568082, 0.0), (-0.00487607, -2.10665, 0.0)]
walletCapacity = 10000
started = False

def placeCollectible(currentTime):
    if len(collectibles) < MAX_COLLECTIBLES:
        position = random.sample(list(positions), 1)
        positions.remove(position[0])
        collectibleDef = "collectible" + str(currentTime)
        collectibleString = "DEF " + collectibleDef + " Ball { translation " + str(position[0][0]) + " " + str(position[0][1]) + " 0.03" + " color 1 0.6667 0 }"
        children.importMFNodeFromString(-1, collectibleString)
        collectibles.append(supervisor.getFromDef(collectibleDef))
    
def printScore():
    print("You have", rupees, "rupees and", dollars, "dollars")
    
def getDistance(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
    
outside = True # To stop the score from printing repeatedly when inside goal
    
def detectCollisions():
    global rupees
    global outside
    global dollars
    global goalPositions
    global started

    robotPosition = robot.getPosition()
    
    for collectible in list(collectibles):
        collectiblePosition = collectible.getPosition()
        
        if getDistance(robotPosition, collectiblePosition) < 0.085:
            collectibles.remove(collectible)
            collectible.remove()
            positions.add((collectiblePosition[0], collectiblePosition[1]))
            rupees += 1000
            if rupees > walletCapacity:
                rupees = walletCapacity
            printScore()
            started = True
            
    for i in range(len(goalPositions)):
        goalPosition = goalPositions[i]
        if getDistance(goalPosition, robotPosition) < 0.15:
            dollars += rupees / goalRates[i]
            rupees = 0
            
            if outside:
                printScore()
            
            outside = False
            break
            
    else:
        outside = True
        
def sendData():
    global collectibles
    global goalPositions
    global goalRates
    robotPosition = robot.getPosition()
    rotationMatrix = robot.getOrientation()
    angle, sign = acos(rotationMatrix[0]) / math.pi * 180, asin(rotationMatrix[3]) / math.pi * 180
    if sign < 0.0:
        angle *= -1
        angle += 360

    data = {}
    data["time"] = gameTime
    data["collectibles"] = []
    data["rupees"] = rupees
    data["dollars"] = dollars
    goalRates[3]
    data["goals"] = [(goalPositions[i][0], goalPositions[i][1], goalRates[i]) for i in range(4)]
    data["robot"] = (robotPosition[0], robotPosition[1])
    data["robotAngleDegrees"] = angle
    
    for collectible in collectibles:
        collectiblePosition = collectible.getPosition()
        data["collectibles"].append((collectiblePosition[0], collectiblePosition[1]))

    emitter.send(json.dumps(data).encode('utf-8'))
        
def customTimestep():
    global started
    global gameTime
    global currentTime
    global rupees
    global goalRates
    global RATE_CHANGE_TIME

    currentTime += timestep
    
    if currentTime == 300000:
        started = True

    if started:
        gameTime += timestep
        
        if gameTime % 992 == 0 and rupees > RUPEE_REDUCTION:
            rupees -= RUPEE_REDUCTION
            
        if gameTime % RATE_CHANGE_TIME == 0:
            goalRates = [random.randint(50, 500) for i in range(4)]
        
while supervisor.step(timestep) != -1:
    detectCollisions()
    placeCollectible(currentTime)
    sendData()
    customTimestep()
