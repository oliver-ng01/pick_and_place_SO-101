import cv2
cap = cv2.VideoCapture(0)
ret,frame = cap.read()
#image = cv2.imread("WIN_20261015_22_26_00_Pro.jpg")

def point_capture(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Coordinate of Image: ({x}, {y})")
        cv2.putText(frame, f'({x}, {y})', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('calib', frame)
cv2.namedWindow('calib', cv2.WINDOW_NORMAL)
cv2.imshow('calib', frame)
cv2.setMouseCallback('calib', point_capture)
cv2.waitKey(0)
cv2.destroyAllWindows()