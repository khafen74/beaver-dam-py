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
            print depth

    out_ras.GetRasterBand(1).WriteArray(out_data)
    out_ras.GetRasterBand(1).SetNoDataValue(0)

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