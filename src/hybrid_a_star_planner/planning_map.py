from scipy.spatial import KDTree
from models import MapParameters


def map():
    obstacleX, obstacleY = [], []

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(0)

    for i in range(51):
        obstacleX.append(0)
        obstacleY.append(i)

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(50)

    for i in range(51):
        obstacleX.append(50)
        obstacleY.append(i)

    for i in range(10, 20):
        obstacleX.append(i)
        obstacleY.append(30)

    for i in range(30, 51):
        obstacleX.append(i)
        obstacleY.append(30)

    for i in range(0, 31):
        obstacleX.append(20)
        obstacleY.append(i)

    for i in range(0, 31):
        obstacleX.append(30)
        obstacleY.append(i)

    for i in range(40, 50):
        obstacleX.append(15)
        obstacleY.append(i)

    for i in range(25, 40):
        obstacleX.append(i)
        obstacleY.append(35)


def calculateMapParameters(obstacleX, obstacleY, xyResolution, yawResolution):
    mapMinX = round(min(obstacleX) / xyResolution)
    mapMaxX = round(max(obstacleX) / xyResolution)
    mapMinY = round(min(obstacleY) / xyResolution)
    mapMaxY = round(max(obstacleY) / xyResolution)

    obstacleKDTree = KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

    return MapParameters(
        mapMinX,
        mapMaxX,
        mapMinY,
        mapMaxY,
        xyResolution,
        yawResolution,
        obstacleKDTree,
        obstacleX,
        obstacleY,
    )
