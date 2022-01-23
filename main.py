from Data.DataLoader import DataLoader
from Data.Util import Util
import numpy as np
import cv2
import math

inputImageName = "lbj.png"
inWidth = 500
inHeight = 500

dataLoader = DataLoader(inWidth, inHeight, inputImageName)
inputContourPoints = dataLoader.getContourPoints()
posePoints = dataLoader.posePoints
poseLines = dataLoader.poseLines

util = Util(inWidth, inHeight)
inputContourPointsRefine = []
for i in range(0, len(inputContourPoints) - 1):
  for point in util.getInterpolatePoints(inputContourPoints[i], inputContourPoints[i+1]):
    inputContourPointsRefine.append(point)


leftUpperArmContour, leftLowerArmContour = util.getPartContour2(poseLines[3], inputContourPointsRefine, 0)
rightUpperArmContour, rightLowerArmContour = util.getPartContour2(poseLines[5], inputContourPointsRefine, 2)
leftUpperLegContour, leftLowerLegContour = util.getPartContour2(poseLines[8], inputContourPointsRefine, 4)
rightUpperLegContour, rightLowerLegContour = util.getPartContour2(poseLines[11], inputContourPointsRefine, 6)
bodyContour = util.getBodyContour([poseLines[3], poseLines[5], poseLines[8], poseLines[11]], inputContourPointsRefine)

workImage = np.zeros((inHeight, inWidth, 3), dtype = np.uint8)

util.drawContour(bodyContour, workImage, dataLoader.inputImageResize)
util.rotatePart2(leftUpperArmContour, leftLowerArmContour, 0, math.pi / 2, -math.pi / 3, workImage, dataLoader.inputImageResize)
util.rotatePart2(rightUpperArmContour, rightLowerArmContour, 2, -math.pi / 2, math.pi / 6, workImage, dataLoader.inputImageResize)
util.rotatePart2(leftUpperLegContour, leftLowerLegContour, 4, -math.pi / 6, -math.pi / 12, workImage, dataLoader.inputImageResize)
util.rotatePart2(rightUpperLegContour, rightLowerLegContour, 6, math.pi / 6, math.pi / 12, workImage, dataLoader.inputImageResize)

# for point in contour:
#     ww, hh = point
#     workImage[hh, ww] = (0, 0, 255)


cv2.imshow('', workImage)
cv2.waitKey(0)





