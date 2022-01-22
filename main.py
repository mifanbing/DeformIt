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


leftArmContour = util.getPartContour(poseLines[3], inputContourPointsRefine)
rightArmContour = util.getPartContour(poseLines[5], inputContourPointsRefine)
leftLegContour = util.getPartContour(poseLines[8], inputContourPointsRefine)
rightLegContour = util.getPartContour(poseLines[11], inputContourPointsRefine)
bodyContour = util.getBodyContour([poseLines[3], poseLines[5], poseLines[8], poseLines[11]], inputContourPointsRefine)

workImage = np.zeros((inHeight, inWidth, 3), dtype = np.uint8)

util.drawContour(bodyContour, workImage, dataLoader.inputImageResize)
util.rotatePart(leftArmContour, 0, math.pi / 3, workImage, dataLoader.inputImageResize)
util.rotatePart(rightArmContour, 1, -math.pi / 3, workImage, dataLoader.inputImageResize)
util.rotatePart(leftLegContour, 2, -math.pi / 6, workImage, dataLoader.inputImageResize)
util.rotatePart(rightLegContour, 3, math.pi / 6, workImage, dataLoader.inputImageResize)

cv2.imshow('', workImage)
cv2.waitKey(0)





