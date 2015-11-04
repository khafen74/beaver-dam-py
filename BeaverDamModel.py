from raster import *
from geometry import *
from stats import *

def createDamPoints(demPath, inFeatPath, outFeatPath):
    driverShp = ogr.GetDriverByName('Esri shapefile')
    in_ds = ogr.Open(inFeatPath)
    out_ds = driverShp.CreateDataSource(outFeatPath)

    if in_ds is None:
        print 'Error opening input shapefile folder'
    if out_ds is None:
        print 'Error opening output shapefile folder'

    damsIn_lyr = in_ds.GetLayerByName('Dams_BRAT_join5_UTM12N')
    bratIn_lyr = in_ds.GetLayerByName('BRAT_TempleFk_WS')
    damsOut_lyr = out_ds.CreateLayer('ModeledDamPoints', bratIn_lyr.GetSpatialRef(), ogr.wkbPoint)

    setDamPointsFieldDefn(damsOut_lyr)

    damSum = 0.0
    nDams = damsIn_lyr.GetFeatureCount()
    newDam_feat = ogr.Feature(damsOut_lyr.GetLayerDefn())

    for i in range(0, nDams, 1):
        damHeight = getDamHeightLognormal(-0.0999, 0.42)[0]
        damSum += damHeight
        oldDam_feat = damsIn_lyr.GetFeature(i)
        nBratFID = oldDam_feat.GetField('ID')
        brat_feat = bratIn_lyr.GetFeature(nBratFID)

        brat_line = brat_feat.GetGeometryRef();
        damPt_old = oldDam_feat.GetGeometryRef();
        az = calcAzimuth(damPt_old.GetX(), damPt_old.GetY(), brat_line.GetX(0), brat_line.GetY(0))

        slope = brat_feat.GetField('iGeo_Slope')
        sampleDist = damHeight/slope
        value = sampleRasterOnLine_LowVal(demPath, damPt_old.GetX(), damPt_old.GetY(), az, sampleDist)

