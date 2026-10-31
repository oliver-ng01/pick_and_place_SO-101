import cv2 as cv
import numpy as np
# Source - https://stackoverflow.com/a/66269158
# Posted by Ali Hashemian
# Retrieved 2026-10-17, License - CC BY-SA 4.0

color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [129, 50, 70]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}

def main(): 
    cap = cv.VideoCapture(0)
    
    while(1):
    
        # Take each frame
        
        _, frame = cap.read()
        cv.imwrite("raw_box.png", frame)
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
        # define range of blue color in HSV
        lower_blue = np.array(color_dict_HSV['blue'][1])
        upper_blue = np.array(color_dict_HSV['blue'][0])

        #range of green
        lower_green = np.array([36, 50, 70])
        upper_green = np.array([89, 255, 255])
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        #t returns a mask the same size as the image, with one number per pixel: 
        #255 if every channel is inside its range, 0 if any channel is outside. Only 0 or 255 
        # (binary in computer vision i guess)
        # Both ends count as inside, so a hue of exactly 110 or 130 is kept.
        
        #ret, thresh = cv.threshold(mask, 127, 255, 0)
        #cv.threshold returns 255 (3rd argument) if pixel > 127, and 0 (4th argument) otherwise
        #ONLY works for greyscale
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            maxcnt = max(contours, key=cv.contourArea)
            cv.drawContours(frame, [maxcnt], -1, (0,255,0), 3)
            x,y,w,h = cv.boundingRect(maxcnt)
            cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

            M = cv.moments(maxcnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv.circle(frame,(cx,cy),3,(255,0,0),thickness=-1)
        # Bitwise-AND mask and original image
        #res = cv.bitwise_and(frame,frame, mask= mask)
    
        cv.imshow('frame',frame)
        #contours are drawn onto the image 'frame'
        cv.imshow('mask',mask)
        #cv.imshow('res',res)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    
    cv.destroyAllWindows()

def getTargetCoordinates(cap):

    _, frame = cap.read()
    
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    
   

    #range of green
    lower_green = np.array([36, 50, 70])
    upper_green = np.array([89, 255, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_green, upper_green)
    
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        maxcnt = max(contours, key=cv.contourArea)
        cv.drawContours(frame, [maxcnt], -1, (0,255,0), 3)
        x,y,w,h = cv.boundingRect(maxcnt)
        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        M = cv.moments(maxcnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    if cx and cy:
        return cx,cy,frame
    else:
        return None,None,None
    
def getBlueBoxCoordinates(cap):

    _, frame = cap.read()
    
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #range of blue
    lower_blue = np.array(color_dict_HSV['blue'][1])
    upper_blue = np.array(color_dict_HSV['blue'][0])    
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        maxcnt = max(contours, key=cv.contourArea)
        cv.drawContours(frame, [maxcnt], -1, (0,255,0), 3)
        x,y,w,h = cv.boundingRect(maxcnt)
        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        M = cv.moments(maxcnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    if cx and cy:
        return cx,cy,frame
    else:
        return None,None,None
if __name__ == "__main__":
    main()