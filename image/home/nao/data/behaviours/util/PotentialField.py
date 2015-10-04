import Global
import Constants

from util.Vector2D import Vector2D, angleBetween
from math import cos, degrees

MAX_ATT = 4
MAX_REP = 800
EPSILON = 1

def getAttractiveField(targetVector, attScale=1):
   dist = max(EPSILON, targetVector.length())
   return targetVector.multiply(MAX_ATT * attScale / dist)

#distThreshold - maximum distance to feel the field
def getRepulsiveField(obsVector, repScale=1, distThresh=800):
   Urep = Vector2D(0, 0)
   dist = obsVector.length()
   # the buffer from your self to the obstacle
   dist = max(EPSILON, dist - Constants.ROBOT_DIAM/2)
   # find new obsPos with modified dist
   obsVector = obsVector.normalise().multiply(dist)
   if dist <= distThresh :
      Urep = obsVector.multiply(MAX_REP * repScale * (1/distThresh - 1/dist)/dist)
   return Urep
