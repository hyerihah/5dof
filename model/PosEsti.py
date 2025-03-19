import math
Re = 6378137
def EstPos(lat1, lng1, bearing, Dist):
    lat2 = math.asin(math.sin(lat1) * math.cos(Dist/Re) + math.cos(lat1) * math.sin(Dist/Re) * math.cos(bearing))
    lng2 = lng1 + math.atan2(math.sin(bearing) * math.sin(Dist/Re) * math.cos(lat1), math.cos(Dist/Re) - math.sin(lat1) * math.sin(lat2))

    return lat2, lng2