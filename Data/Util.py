import numpy as np
import math
import cv2

class Util:
    def __init__(self, inWidth, inHeight):
        self.inWidth = inWidth
        self.inHeight = inHeight
        self.cutContours = [[]] * 8
        
    def findStartAndEnd(self, contourPoints, targetPoint, startPoint, endPoint):
      w, h = targetPoint
      wStart, hStart = startPoint
      wEnd, hEnd = endPoint
    
      left = -1
      right = self.inWidth
      startIndex = -1
      endIndex = -1
    
      index45Degree = []
    
      for i in range(len(contourPoints)):
        point = contourPoints[i]
        wPoint, hPoint = point
    
        length1 = math.sqrt((hEnd - hStart) ** 2 + (wEnd - wStart) ** 2)
        length2 = math.sqrt((hPoint - h) ** 2 + (wPoint - w) ** 2)
        if length2 == 0:
          continue
        dotProduct = (hEnd - hStart) * (hPoint - h) + (wEnd - wStart) * (wPoint - w)
        angle = abs(np.arccos(dotProduct / length1 / length2))
        if abs(angle - math.pi / 2) < math.pi / 20:
          if startIndex == -1:
            startIndex = i
          else:
            endIndex = i
        else:
          if startIndex != -1 and endIndex != -1:
            index45Degree.append((startIndex, endIndex))
            startIndex = -1
            endIndex = -1
    
      index45DegreeRefine = []
      left = -1
      right = self.inWidth
      index45DegreeLeft = -1, -1
      index45DegreeRight = -1, -1
      for pair in index45Degree:
        start2, end2 = pair
        ww, hh = contourPoints[start2]
        if ww < w and ww > left:
          left = contourPoints[start2][0]
          index45DegreeLeft = pair
        if ww > w and ww < right:
          right = contourPoints[start2][0]
          index45DegreeRight = pair
    
      index45DegreeRefine = [index45DegreeLeft, index45DegreeRight] if index45DegreeLeft[0] < index45DegreeRight[0] else [index45DegreeRight, index45DegreeLeft]
      return index45DegreeRefine
    
    def findLeftAndRightOld(self, contourPoints, targetPoint):
      w, h = targetPoint
      left = -1
      right = self.inWidth
    
      for point in contourPoints:
        if abs(point[1] - h) < 3:
    
          if point[0] < w and point[0] > left:
            left = point[0]
    
          if point[0] > w and point[0] < right:
            right = point[0]
    
      if left == -1:
        left = w
      if right == self.inWidth:
        right = w
    
      return left, right
    
    def getInterpolatePoints(self, pointStart, pointEnd):
      wStart, hStart = pointStart
      wEnd, hEnd = pointEnd
    
      points = []
    
      if abs(wStart - wEnd) > abs(hStart - hEnd):
        step =  1 if wStart < wEnd else -1
    
        for w in range(wStart, wEnd, step):
          k = (w - wStart) / (wEnd - wStart)
          h = round(hStart + k * (hEnd - hStart))
          points.append((w, h))
      else:
        step =  1 if hStart < hEnd else -1
    
        for h in range(hStart, hEnd, step):
          k = (h - hStart) / (hEnd - hStart)
          w = round(wStart + k * (wEnd - wStart))
          points.append((w, h))
    
      return points
      
    def getPartContour(self, poseLine, contourPoints):
        pointPartStart, pointPartEnd = poseLine
        rangeStartAndEnd = self.findStartAndEnd(contourPoints, pointPartStart, pointPartStart, pointPartEnd)
        
        indexPartContourStart = rangeStartAndEnd[0][0]
        indexPartContourEnd = rangeStartAndEnd[1][0]
        
        partCutContour = self.getInterpolatePoints(contourPoints[indexPartContourStart], contourPoints[indexPartContourEnd])
        
        partContour = []
        if indexPartContourEnd - indexPartContourStart < len(contourPoints) / 2:
          partContour = partCutContour + contourPoints[indexPartContourStart:indexPartContourEnd]
        else:
          partContour = partCutContour + contourPoints[indexPartContourEnd:] + contourPoints[:indexPartContourStart]
        
        return partContour

    def getPartContour2(self, poseLine, contourPoints, index):
        pointPartStart, pointPartEnd = poseLine
        
        startPointRange = self.findStartAndEnd(contourPoints, pointPartStart, pointPartStart, pointPartEnd)
        startPointStartIndex = startPointRange[0][0]
        startPointEndIndex = startPointRange[1][0]
        startPointCutContour = self.getInterpolatePoints(contourPoints[startPointStartIndex], contourPoints[startPointEndIndex])
        
        endPointRange = self.findStartAndEnd(contourPoints, pointPartEnd, pointPartStart, pointPartEnd)
        endPointStartIndex = endPointRange[0][0]
        endPointEndIndex = endPointRange[1][0]
        endPointCutContour = self.getInterpolatePoints(contourPoints[endPointStartIndex], contourPoints[endPointEndIndex])
        
        self.cutContours[index] = startPointCutContour
        self.cutContours[index + 1] = endPointCutContour
        
        partUpperContour = []
        partUpperContour.extend(startPointCutContour)
        
        # print("startPoint startIndex: %d endIndex: %d" % (startPointStartIndex, startPointEndIndex))
        # print("endPoint startIndex: %d endIndex: %d" % (endPointStartIndex, endPointEndIndex))
        
        if startPointEndIndex - startPointStartIndex < len(contourPoints) / 2:
          partUpperContour.extend(contourPoints[endPointEndIndex:startPointEndIndex])
          partUpperContour.extend(endPointCutContour)
          partUpperContour.extend(contourPoints[startPointStartIndex:endPointStartIndex])
        else:
          if endPointEndIndex < startPointStartIndex:
              partUpperContour.extend(contourPoints[startPointEndIndex:])
              partUpperContour.extend(contourPoints[:endPointStartIndex])
              partUpperContour.extend(endPointCutContour)
              partUpperContour.extend(contourPoints[endPointEndIndex:startPointStartIndex])
          elif endPointStartIndex > startPointEndIndex: 
              partUpperContour.extend(contourPoints[startPointEndIndex:endPointStartIndex])
              partUpperContour.extend(endPointCutContour)
              partUpperContour.extend(contourPoints[endPointEndIndex:])
              partUpperContour.extend(contourPoints[:startPointStartIndex])
          else: 
              partUpperContour.extend(contourPoints[startPointEndIndex:endPointEndIndex])
              partUpperContour.extend(endPointCutContour)
              partUpperContour.extend(contourPoints[endPointStartIndex:startPointStartIndex])
        
        partLowerContour = []
        partLowerContour.extend(endPointCutContour)
        if endPointEndIndex - endPointStartIndex < len(contourPoints) / 2:
            partLowerContour.extend(contourPoints[endPointStartIndex:endPointEndIndex])
        else: 
            partLowerContour.extend(contourPoints[endPointEndIndex:])
            partLowerContour.extend(contourPoints[:endPointStartIndex])
              
        return partUpperContour, partLowerContour
    
    def getBodyContour(self, poseLines, contourPoints):
        def inInterval(target, start, end, length):
          if end - start < length / 2:
            return target > start and target < end
          else:
            return target > end or target < start

        intervals = []
        cutContours = []
        for poseLine in poseLines:
            pointPartStart, pointPartEnd = poseLine
            rangeStartAndEnd = self.findStartAndEnd(contourPoints, pointPartStart, pointPartStart, pointPartEnd)
            intervals.append((rangeStartAndEnd[0][0], rangeStartAndEnd[1][0]))
            cutContours.append(self.getInterpolatePoints(contourPoints[rangeStartAndEnd[0][0]], contourPoints[rangeStartAndEnd[1][0]]))
        
        
        trimmedBodyContour = []
        hasAddedParts = [False, False, False, False]
        for i in range(len(contourPoints)):
            isBody = True
            for j in range(len(hasAddedParts)):
                if inInterval(i, intervals[j][0], intervals[j][1], len(contourPoints)):
                    isBody = False
                    if not hasAddedParts[j]:
                        hasAddedParts[j] = True
                        trimmedBodyContour.extend(cutContours[j])
            if isBody:
                trimmedBodyContour.append(contourPoints[i])
                
        return trimmedBodyContour

    def drawContour(self, contour, workImage, inputImage):
      hMin = self.inHeight
      hMax = -1
      for point in contour:
        w, h = point
        if h < hMin:
          hMin = h
        if h > hMax:
          hMax = h
    
      for h in range(hMin, hMax):
        wMin = self.inWidth
        wMax = -1
        for point in contour:
          w, h2 = point
          if h2 == h:
            if w < wMin:
              wMin = w
            if w > wMax:
              wMax = w
        for w in range(wMin, wMax):
          workImage[h, w] = inputImage[h, w]         
    
  
    def drawContourWithRotation(self, contour, M, MInv, w0, h0, workImage, inputImage):
      hMin = self.inHeight
      hMax = -1
    
      contourRotate = []
      for point in contour:
        w, h = point
        wRotate = int((w - w0) * M[0][0] + (h - h0) * M[0][1] + w0)
        hRotate = int((w - w0) * M[1][0] + (h - h0) * M[1][1] + h0)
        contourRotate.append((wRotate, hRotate))
        if hRotate < hMin:
          hMin = hRotate
        if hRotate > hMax:
          hMax = hRotate
    
      contourRotateRefine = []
      for i in range(0, len(contourRotate) - 1):
        for point in self.getInterpolatePoints(contourRotate[i], contourRotate[i+1]):
          contourRotateRefine.append(point)
    
      for h in range(hMin, hMax):
        wMin = self.inWidth
        wMax = -1
        for point in contourRotateRefine:
          w, h2 = point
          if h2 == h:
            if w < wMin:
              wMin = w
            if w > wMax:
              wMax = w
        for w in range(wMin, wMax):
          w2 = int((w - w0) * MInv[0][0] + (h - h0) * MInv[0][1] + w0)
          h2 = int((w - w0) * MInv[1][0] + (h - h0) * MInv[1][1] + h0)
          workImage[h, w] = inputImage[h2, w2] 

    def drawContourWithRotation2(self, contour, M1, M1Inv, w1, h1, M2, M2Inv, w2, h2, workImage, inputImage):
      hMin = self.inHeight
      hMax = -1
    
      contourRotate = []
      for point in contour:
        w, h = point
        wRotate = int((w - w1) * M1[0][0] + (h - h1) * M1[0][1] + w1)
        hRotate = int((w - w1) * M1[1][0] + (h - h1) * M1[1][1] + h1)
        wRotate2 = int((wRotate - w2) * M2[0][0] + (hRotate - h2) * M2[0][1] + w2)
        hRotate2 = int((wRotate - w2) * M2[1][0] + (hRotate - h2) * M2[1][1] + h2)
        contourRotate.append((wRotate2, hRotate2))
        if hRotate2 < hMin:
          hMin = hRotate2
        if hRotate2 > hMax:
          hMax = hRotate2
    
        contourRotateRefine = []
        for i in range(0, len(contourRotate) - 1):
          for point in self.getInterpolatePoints(contourRotate[i], contourRotate[i+1]):
            contourRotateRefine.append(point)

      for h in range(hMin, hMax):
        wMin = self.inWidth
        wMax = -1
        for point in contourRotate:
          ww, hh = point
          if hh == h:
            if ww < wMin:
              wMin = ww
            if ww > wMax:
              wMax = ww
        for w in range(wMin, wMax):
          wM2Inv = int((w - w2) * M2Inv[0][0] + (h - h2) * M2Inv[0][1] + w2)
          hM2Inv = int((w - w2) * M2Inv[1][0] + (h - h2) * M2Inv[1][1] + h2)
          wM1Inv = int((wM2Inv - w1) * M1Inv[0][0] + (hM2Inv - h1) * M1Inv[0][1] + w1)
          hM1Inv = int((wM2Inv - w1) * M1Inv[1][0] + (hM2Inv - h1) * M1Inv[1][1] + h1)
          workImage[h, w] = inputImage[hM1Inv, wM1Inv] 
           
    def find45degree(self, contourPoints, targetPoint, startPoint, endPoint):
      w, h = targetPoint
      wStart, hStart = startPoint
      wEnd, hEnd = endPoint
    
      startIndex = -1
      endIndex = -1
    
      index45Degrees = []
    
      for i in range(len(contourPoints)):
        point = contourPoints[i]
        wPoint, hPoint = point
    
        length1 = math.sqrt((hEnd - hStart) ** 2 + (wEnd - wStart) ** 2)
        length2 = math.sqrt((hPoint - h) ** 2 + (wPoint - w) ** 2)
        if length2 == 0:
          continue
        dotProduct = (hEnd - hStart) * (hPoint - h) + (wEnd - wStart) * (wPoint - w)
        angle = np.arccos(dotProduct / length1 / length2)
        if abs(angle - math.pi / 3) < math.pi / 20:
          if startIndex == -1:
            startIndex = i
          else:
            endIndex = i
        else:
          if startIndex != -1 and endIndex != -1:
            index45Degrees.append((startIndex, endIndex))
            startIndex = -1
            endIndex = -1
    
      #print(index45Degrees)
      index45DegreeRefine = index45Degrees[0][1]
      ww, hh = contourPoints[index45DegreeRefine]
      distMin = (ww - w) ** 2 + (hh - h) ** 2
      for index45Degree in index45Degrees:
        w3, h3 = contourPoints[index45Degree[1]]
        distTemp = (w3 - w) ** 2 + (h3 - h) ** 2
        if distTemp < distMin:
          index45DegreeRefine = index45Degree[1]
    
      return index45DegreeRefine   

    def rotatePart(self, partContour, index, angle, workImage, inputImage):
        cutContour = self.cutContours[index]
        index45Degree = self.find45degree(partContour, partContour[0], partContour[0], partContour[len(cutContour) - 1])
        
        pointA = partContour[index45Degree]
        triangle = []
        if abs(index45Degree - len(cutContour)) > len(partContour) / 2 :
          triangle.extend(cutContour)
          triangle.extend(partContour[index45Degree:])
          triangle.extend(partContour[:len(cutContour) - 1])
        else:
          triangle.extend(partContour[:index45Degree])
        
        newLine = self.getInterpolatePoints(pointA, cutContour[0])
        triangle.extend(newLine)
        
        w1, h1 = cutContour[0]
        w2, h2 = cutContour[-1]
        w3, h3 = pointA
        inputPair = np.array( [[0, 0], [w2 - w1, h2 - h1], [w3 - w1, h3 - h1]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [w2 - w1, h2 - h1], [math.cos(angle) * (w3 - w1) + math.sin(angle) * (h3 - h1), -math.sin(angle) * (w3 - w1) + math.cos(angle) * (h3 - h1)]] ).astype(np.float32)
        
        M = cv2.getAffineTransform(inputPair, refPair)
        MInv = cv2.getAffineTransform(refPair, inputPair)
        self.drawContourWithRotation(triangle, M, MInv, w1, h1, workImage, inputImage)
        
        partWith45DegreeCut = []
        partWith45DegreeCut.extend(partContour[len(cutContour) : index45Degree])
        partWith45DegreeCut.extend(newLine)
        inputPair = np.array( [[0, 0], [0, -1], [-1, 0]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [-math.sin(angle), -math.cos(angle)], [-math.cos(angle), math.sin(angle)]] ).astype(np.float32)
        
        M = cv2.getAffineTransform(inputPair, refPair)
        MInv = cv2.getAffineTransform(refPair, inputPair)
        self.drawContourWithRotation(partWith45DegreeCut, M, MInv, w1, h1, workImage, inputImage)       
        
    def rotatePart2(self, partUpperContour, partLowerContour, index, angleUpper, angleLower, workImage, inputImage):
        upperCutContour = self.cutContours[index]
        index45Degree = self.find45degree(partUpperContour, partUpperContour[0], partUpperContour[0], partUpperContour[len(upperCutContour) - 1])
        
        pointA = partUpperContour[index45Degree]
        triangle = []
        if abs(index45Degree - len(upperCutContour)) > len(partUpperContour) / 2 :
          triangle.extend(upperCutContour)
          triangle.extend(partUpperContour[index45Degree:])
          triangle.extend(partUpperContour[:len(upperCutContour) - 1])
        else:
          triangle.extend(partUpperContour[:index45Degree])
        
        newLine = self.getInterpolatePoints(pointA, upperCutContour[0])
        triangle.extend(newLine)
        
        w1, h1 = upperCutContour[0]
        w2, h2 = upperCutContour[-1]
        w3, h3 = pointA
        inputPair = np.array( [[0, 0], [w2 - w1, h2 - h1], [w3 - w1, h3 - h1]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [w2 - w1, h2 - h1], [math.cos(angleUpper) * (w3 - w1) + math.sin(angleUpper) * (h3 - h1), -math.sin(angleUpper) * (w3 - w1) + math.cos(angleUpper) * (h3 - h1)]] ).astype(np.float32)
        
        M = cv2.getAffineTransform(inputPair, refPair)
        MInv = cv2.getAffineTransform(refPair, inputPair)
        self.drawContourWithRotation(triangle, M, MInv, w1, h1, workImage, inputImage)
        
        partWith45DegreeCut = []
        partWith45DegreeCut.extend(partUpperContour[index45Degree:])
        partWith45DegreeCut.extend(newLine)
        inputPair = np.array( [[0, 0], [0, -1], [-1, 0]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [-math.sin(angleUpper), -math.cos(angleUpper)], [-math.cos(angleUpper), math.sin(angleUpper)]] ).astype(np.float32)
        
        M = cv2.getAffineTransform(inputPair, refPair)
        MInv = cv2.getAffineTransform(refPair, inputPair)
        self.drawContourWithRotation(partWith45DegreeCut, M, MInv, w1, h1, workImage, inputImage)          
        
        #------
        lowerCutContour = self.cutContours[index + 1]
        index45Degree = self.find45degree(partLowerContour, partLowerContour[0], partLowerContour[0], partLowerContour[len(lowerCutContour) - 1])
        
        pointA = partLowerContour[index45Degree]
        triangle = []
        if abs(index45Degree - len(lowerCutContour)) > len(partLowerContour) / 2 :
          triangle.extend(lowerCutContour)
          triangle.extend(partLowerContour[index45Degree:])
          triangle.extend(partLowerContour[:len(lowerCutContour) - 1])
        else:
          triangle.extend(partLowerContour[:index45Degree])
        
        newLine = self.getInterpolatePoints(pointA, lowerCutContour[0])
        triangle.extend(newLine)
        
        w1Lower, h1Lower = lowerCutContour[0]
        w2Lower, h2Lower = lowerCutContour[-1]
        w3Lower, h3Lower = pointA
        inputPair = np.array( [[0, 0], [w2Lower - w1Lower, h2Lower - h1Lower], [w3Lower - w1Lower, h3Lower - h1Lower]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [w2Lower - w1Lower, h2Lower - h1Lower], [math.cos(angleLower) * (w3Lower - w1Lower) + math.sin(angleLower) * (h3Lower - h1Lower), -math.sin(angleLower) * (w3Lower - w1Lower) + math.cos(angleLower) * (h3Lower - h1Lower)]] ).astype(np.float32)
        
        w1Rotate = int((w1Lower - w1) * M[0][0] + (h1Lower - h1) * M[0][1] + w1)
        h1Rotate = int((w1Lower - w1) * M[1][0] + (h1Lower - h1) * M[1][1] + h1)
        M2 = cv2.getAffineTransform(inputPair, refPair)
        M2Inv = cv2.getAffineTransform(refPair, inputPair)
        self.drawContourWithRotation2(triangle, M, MInv, w1, h1, M2, M2Inv, w1Rotate, h1Rotate, workImage, inputImage)        
        
        
        partWith45DegreeCut = []
        if index45Degree > len(partLowerContour) / 2:
            partWith45DegreeCut.extend(partLowerContour[:index45Degree])
        else:
            partWith45DegreeCut.extend(partLowerContour[index45Degree:])
        partWith45DegreeCut.extend(newLine)
        w4, h4 = lowerCutContour[0]
        w4Rotate = int((w4 - w1) * M[0][0] + (h4 - h1) * M[0][1] + w1)
        h4Rotate = int((w4 - w1) * M[1][0] + (h4 - h1) * M[1][1] + h1)
        inputPair = np.array( [[0, 0], [0, -1], [-1, 0]] ).astype(np.float32)
        refPair = np.array( [[0, 0], [-math.sin(angleLower), -math.cos(angleLower)], [-math.cos(angleLower), math.sin(angleLower)]] ).astype(np.float32)
        
        M2 = cv2.getAffineTransform(inputPair, refPair)
        M2Inv = cv2.getAffineTransform(refPair, inputPair)
        
        self.drawContourWithRotation2(partWith45DegreeCut, M, MInv, w1, h1, M2, M2Inv, w4Rotate, h4Rotate, workImage, inputImage)
        
        
        
        
        
        
        
        
        
        
        
    
  
    
  
    
  
    
  