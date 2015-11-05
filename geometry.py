from osgeo import ogr
from osgeo import osr
import math
from shapely.geometry import point, polygon
from shapely.wkb import loads

def addDegrees(base, addValue):
    value = base + addValue
    if value > 360.0:
        remainder = value - 360.0
        value = remainder
    elif value < 0.0:
        remainder = abs(value)
        value = 360.0 - remainder
    if value > 360.0 or value < 0.0:
        print 'Error adding degrees'

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
        print 'Error calculating azimuth: start x,y = %s, %s - end x,y = %s, %s' % (startX, startY, endX, endY)
        azimuth = 0.0

    return azimuth

def calcCoords(startX, startY, azimuth, distance):
    if (azimuth > 0.0) and (azimuth < 90.0):
        theta = azimuth
        deltaY = math.sin(theta*(math.pi/180.0))*distance
        deltaX = math.cos(theta*(math.pi/180.0))*distance

    elif (azimuth > 90.0) and (azimuth < 180.0):
        theta = azimuth - 90.0
        deltaX = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaY = math.cos(theta*(math.pi/180.0))*distance

    elif (azimuth > 180.0) and (azimuth < 270.0):
        theta = azimuth - 180.0
        deltaY = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaX = math.cos(theta*(math.pi/180.0))*distance*(-1.0)

    elif (azimuth > 270.0) and (azimuth < 360.0):
        theta = 360.0 - azimuth
        deltaY = math.sin(theta*(math.pi/180.0))*distance*(-1.0)
        deltaX = math.cos(theta*(math.pi/180.0))*distance

    else:
        print 'Error calculating coordinates ' + str(azimuth)

    newX = startX + deltaX
    newY = startY + deltaY

    return newX, newY

def pointsInPolygon(inPath, outPath, ptsLyrName, polyLyrName):
    driverShp = ogr.GetDriverByName('Esri shapefile')
    in_ds = driverShp.CreateDataSource(inPath)
    out_ds = driverShp.CreateDataSource(outPath)

    if out_ds is None:
        print 'Error opening output shapefile folder'
    if in_ds is None:
        print 'Error opening input shapefile folder'

    pt_lyr = in_ds.GetLayerByName(ptsLyrName)
    poly_lyr = out_ds.GetLayerByName(polyLyrName)
    pndPts_lyr = out_ds.CreateLayer('PondPts', pt_lyr.GetSpatialRef(), ogr.wkbPoint)
    setPondPointsFieldDefn(pndPts_lyr)
    print poly_lyr.GetFeatureCount()

    count = 0.0
    while 1:
        poly_feat = poly_lyr.GetNextFeature()
        if not poly_feat:
            break

        poly_sh = loads(poly_feat.GetGeometryRef().ExportToWkb())
        found = False
        count2 = 0.0
        pt_lyr.ResetReading()
        for pt_feat in pt_lyr:
            pt_geom = pt_feat.GetGeometryRef()
            pt_sh = loads(pt_geom.ExportToWkb())
            count2 += 1
            if poly_sh.contains(pt_sh):
                newPt_feat = ogr.Feature(pndPts_lyr.GetLayerDefn())
                newPt_feat.SetField('dam_elev', poly_feat.GetField('d_elev'))
                newPt_feat.SetField('elev', pt_feat.GetField('elev'))
                newPt_feat.SetGeometry(pt_geom)
                pndPts_lyr.CreateFeature(newPt_feat)
                found = True

        count += 1
        print count2
        if found:
            print count



def setDamPointsFieldDefn(layer):
    field = ogr.FieldDefn('endx', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('endy', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('az_us', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('g_elev', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('d_elev', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('slope', ogr.OFTReal)
    layer.CreateField(field)

def setDamFeature(feat, point, elev, damHeight, slope, x, y):
    feat.SetField('g_elev', float(elev))
    feat.SetField('d_elev', elev+damHeight)
    feat.SetField('slope', slope)
    feat.SetField('endx', x)
    feat.SetField('endy', y)
    feat.SetField('az_us', calcAzimuth(point.GetX(), point.GetY(), x, y))
    feat.SetGeometry(point)

def setPondPointsFieldDefn(layer):
    field = ogr.FieldDefn('dam_elev', ogr.OFTReal)
    layer.CreateField(field)
    field = ogr.FieldDefn('elev', ogr.OFTReal)
    layer.CreateField(field)

