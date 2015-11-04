from osgeo import ogr
from osgeo import osr
import math

def addDegrees(base, addValue):
    value = base + addValue
    if value > 360.0:
        remainder = value - 360.0
        value = remainder
    elif value < 0.0:
        remainder = abs(value)
        value = 360.0 - remainder

    return value

def angleBetweenLines(x1, y1, x2, y2, x3, y3):
    angle = math.atan2(y1.y3, x1-x3) - math.atan2(y2-y3, x2-x3)

    while angle < -math.pi:
        angle += 2*math.pi

    while angle > math.pi:
        angle -= 2*math.pi

    return angle

def calcAzimuth(startX, startY, endX, endY):
    if (startX > endX) and (startY > endY):
        theta = math.atan2((startY-endY), (startX-endX)) * 180.0 / math.pi
        azimuth = 180.0 + theta
    elif (startX < endX) and (startY > endY):
        theta = math.atan2((startY-endY), (endX-startX)) * 180.0 / math.pi
        azimuth = 360.0 + theta
    elif (startX < endX) and (startY < endY):
        theta = math.atan2((endY-startY), (endX-startX)) * 180.0 / math.pi
        azimuth = theta
    elif (startX > endX) and (startY < endY):
        theta = math.atan2((startX-endX), (startY-endY)) * 180.0 / math.pi
        azimuth =  90.0 + theta
    else:
        #error
        print 'Error calculating azimuth, may have a straight line'

    return azimuth

def calcCoords(startX, startY, azimuth, distance):
    if (azimuth > 0.0) and (azimuth < 90.0):
        theta = azimuth
        deltaY = math.sin(theta*(math.pi/180.0))*distance
        deltaX = math.cos(theta*(math.pi/180.0))*distance
    elif (azimuth > 90.0) and (azimuth < 180.0):
        theta = azimuth
        deltaX = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaY = math.cos(theta*(math.pi/180.0))*distance
    elif (azimuth > 180.0) and (azimuth < 270.0):
        theta = azimuth
        deltaY = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaX = math.cos(theta*(math.pi/180.0))*distance*(-1.0)
    elif (azimuth > 270.0) and (azimuth < 360.0):
        theta = azimuth
        deltaY = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaX = math.cos(theta*(math.pi/180.0))*distance

    newX = startX + deltaX
    newY = startY + deltaY

    return newX, newY

def setDamPointsFieldDefn(layer):
    field = ogr.FieldDefn('endx', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('endy', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('az_us', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('g_elev', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('d_eelv', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('slope', ogr.OFTReal)
    layer.CreateField(field)
