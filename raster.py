from osgeo import gdal
import numpy as np
from geometry import *

def cleanInundationRaster(raster):
    newRaster = np.empty(raster.shape, np.float)
    newRaster = np.where(raster <= 0.0, -9999, raster)
    return newRaster

def getRasterValueAtPoint(ds, x, y):
    geot = ds.GetGeoTransform()
    inv_geot = gdal.InvGeoTransform(geot)
    col, row = gdal.ApplyGeoTransform(inv_geot, x, y)

    band = ds.GetRasterBand(1).ReadAsArray()
    value = band[row, col]

    return value

def sampleRasterOnLine_LowVal(rasterPath, startX, startY, azimuth, distance):
    az1 = addDegrees(azimuth, 90.0)
    az2 = addDegrees(azimuth, -90.0)

    in_ds = gdal.Open(rasterPath)
    geot = in_ds.GetGeoTransform()

    interval = geot[1]
    nSamples = int(math.ceil(distance/interval))

    for i in range(0, nSamples, 1):
        newX, newY = calcCoords(startX, startY, az1, interval*(i+1))
        val = getRasterValueAtPoint(in_ds, newX, newY)

        if i==0:
            lowVal = val
            x = newX
            y = newY
        else:
            if val < lowVal:
                lowVal = val
                x = newX
                y = newY

        newX, newY = calcCoords(startX, startY, az2, interval*(i+1))
        val = getRasterValueAtPoint(in_ds, newX, newY)

        if val < lowVal:
                lowVal = val
                x = newX
                y = newY

    return lowVal, x, y