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
    newDam_pt = ogr.Geometry(ogr.wkbPoint)

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
        elev, x, y = sampleRasterOnLine_LowVal(demPath, damPt_old.GetX(), damPt_old.GetY(), az, sampleDist)
        newDam_pt.AddPoint(x, y)
        value, x, y = sampleRasterOnLine_LowVal(demPath, brat_line.GetX(0), brat_line.GetY(0), az, sampleDist)
        setDamFeature(newDam_feat, newDam_pt, elev, damHeight, slope, x, y)
        damsOut_lyr.CreateFeature(newDam_feat)

    print damSum/nDams

def createSearchPolygons(outFeatPath):
    ANGLE_OFFSET = [-90.0, -45.0, 0.0, 45.0, 90.0]
    driverShp = ogr.GetDriverByName('Esri shapefile')
    out_ds = driverShp.CreateDataSource(outFeatPath)

    if out_ds is None:
        print 'Error opening output shapefile folder'

    damPts_lyr = out_ds.GetLayerByName('ModeledDamPoints')
    sPoly_lyr = out_ds.CreateLayer('DamSearchPolygons', damPts_lyr.GetSpatialRef(), ogr.wkbPolygon)
    field = ogr.FieldDefn('d_elev', ogr.OFTReal)
    sPoly_lyr.CreateField(field)

    damPts_lyr.ResetReading()
    count = 0

    for damPt_feat in damPts_lyr:
        sPoly_feat = ogr.Feature(sPoly_lyr.GetLayerDefn())
        sPoly_feat.SetField('d_elev', damPt_feat.GetField('d_elev'))
        azStart = damPt_feat.GetField('az_us')
        slope = damPt_feat.GetField('slope')
        dist = (damPt_feat.GetField('d_elev')-damPt_feat.GetField('g_elev'))/slope
        dam_pt = damPt_feat.GetGeometryRef()
        poly = ogr.Geometry(ogr.wkbPolygon)
        ring = ogr.Geometry(ogr.wkbLinearRing)

        for offset in ANGLE_OFFSET:
            azCur = addDegrees(azStart, offset)
            x, y = calcCoords(dam_pt.GetX(), dam_pt.GetY(), azCur, dist)
            ring.AddPoint(x, y)

        #ring.AddPoint(dam_pt.GetX(), dam_pt.GetY())
        poly.AddGeometry(ring)
        poly.CloseRings()
        sPoly_feat.SetGeometry(poly)
        sPoly_lyr.CreateFeature(sPoly_feat)

        count +=1

    sPoly_lyr.SyncToDisk()

    print count
