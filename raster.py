from osgeo import gdal
import numpy as np
from geometry import *

def calcArea(array, noData, geot):
    array = np.where(array != noData, 1, 0)
    sum = array.sum()
    area = sum * geot[1]**2
    return area

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

def pointToRaster(inRasPath, outRasPath, dsPath, layerName):
    driverShp = ogr.GetDriverByName('Esri shapefile')
    in_ds = driverShp.CreateDataSource(dsPath)

    if in_ds is None:
        print 'Error opening input shapefile folder'

    in_ras = gdal.Open(inRasPath)
    out_ras = gdal.GetDriverByName('GTiff').Create(outRasPath, in_ras.RasterXSize, in_ras.RasterYSize, 1, gdal.GDT_Float32)
    geot = in_ras.GetGeoTransform()
    out_ras.SetGeoTransform(geot)
    inv_geot = gdal.InvGeoTransform(geot)
    in_data = in_ras.GetRasterBand(1).ReadAsArray()
    out_data = np.zeros(in_data.shape, np.float)

    pt_lyr = in_ds.GetLayerByName(layerName)

    pt_lyr.ResetReading()

    for pt in pt_lyr:
        pt_geom = pt.GetGeometryRef()
        depth = pt.GetField('dam_elev') - pt.GetField('elev')
        if depth > 0:
            col, row = gdal.ApplyGeoTransform(inv_geot, pt_geom.GetX(), pt_geom.GetY())
            out_data[row, col] = depth

    out_ras.GetRasterBand(1).WriteArray(out_data)
    out_ras.GetRasterBand(1).SetNoDataValue(0)
    area = calcArea(out_data, 0.0, geot)
    del in_ras, out_ras
    return area

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

    del in_ds

    return lowVal, x, y

def updateInundationRaster(inPath, depthPath):
    dep_ras = gdal.Open(depthPath)
    in_ras = gdal.Open(inPath, gdal.GA_Update)

    dep_band = dep_ras.GetRasterBand(1)
    in_band = in_ras.GetRasterBand(1)

    dep_data = dep_band.ReadAsArray()
    in_data = in_band.ReadAsArray()

    in_data = np.where(dep_data > 0.0, in_data+1, in_data)

    in_band.WriteArray(in_data)

