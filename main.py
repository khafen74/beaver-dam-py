from raster import *
from geometry import *
from stats import *
from BeaverDamModel import *
import os

def cleanup(directory):
    #delete files in diretory
    os.chdir(directory)
    for file in os.listdir('.'):
        os.remove(file)

#def runSingle():
    #run single model simulation

#def runMonteCarlo(dataFiles, nIterations):
    #run simulation 1000 times with stochastic dam heights

def setDataFiles(nResolution):
    dataFiles = []
    dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/01_shpIn')
    dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/03_shpOut')

    if nResolution == 1:
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/02_rasIn/fme450000.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/ponddepth_1m.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/freqwet_1m.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/freqwet_1m.csv')
        dataFiles.append('dempoints_1m_clip2')
    elif nResolution == 10:
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/02_rasIn/templefk_10m_ws.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/ponddepth_10m.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/freqwet_10m.tif')
        dataFiles.append('E:/etal/Projects/NonLoc/Beaver_Modeling/02_Data/z_TestRuns/04_rasOut/freqwet_10m.csv')
        dataFiles.append('dempoints_10m_clip2')

    return dataFiles

#-------------------------------------------------------
#Executes Model
#-------------------------------------------------------
dataFiles = setDataFiles(10)
cleanup(dataFiles[1])
createDamPoints(dataFiles[2], dataFiles[0], dataFiles[1])
createSearchPolygons(dataFiles[1])
pointsInPolygon(dataFiles[0], dataFiles[1], dataFiles[6], 'DamSearchPolygons')
pointToRaster(dataFiles[2], dataFiles[3], dataFiles[1], 'PondPts')

print 'Done!'