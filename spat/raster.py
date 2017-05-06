import pyproj as proj
from osgeo import gdalnumeric, gdal

def world2Pixel(geoMatrix, x, y):
    """
    Uses a gdal geomatrix (gdal.GetGeoTransform()) to calculate
    the pixel location of a geospatial coordinate
    """
    ulX = geoMatrix[0]
    ulY = geoMatrix[3]
    xDist = geoMatrix[1]
    yDist = geoMatrix[5]
    rtnX = geoMatrix[2]
    rtnY = geoMatrix[4]
    pixel = int((x - ulX) / xDist)
    line = int((ulY - y) / xDist)
    #print pixel, line
    return (pixel, line)

class RasterImage:
    def __init__(self, path, src_proj = 'epsg:2950'):
        self.srcArray = gdalnumeric.LoadFile(path)
        self.srcImage = gdal.Open(path)
        self.geoTrans = self.srcImage.GetGeoTransform()
        self.srcProj = proj.Proj(init=src_proj)

    def at(self, coord, dst_proj):
        x, y = coord
        coord = proj.transform(self.srcProj, dst_proj, x, y)
        pixel, line = world2Pixel(self.geoTrans, coord[0], coord[1])
        return self.srcArray[line][pixel]