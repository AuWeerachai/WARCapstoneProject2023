import cv2
import numpy as np
import conveyor_lib


cap = cv2.VideoCapture(0)

# Conveyor belt library
relay = conveyor_lib.Conveyor() 


while True:
    _, frame = cap.read()
    
    # Belt
    belt = frame[209: 327, 137: 280]
    gray_belt = cv2.cvtColor(belt, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray_belt, 80, 255, cv2.THRESH_BINARY)
    
    # Detect the Nuts
    _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        # Calculate area
        area = cv2.contourArea(cnt)
        
        # Distinguish small and big nuts
        if area > 400:
            # big nut
            cv2.rectangle(belt, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # Stop belt
            relay.turn_off()
            
        elif 100 < area < 400:
            cv2.rectangle(belt, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
        cv2.putText(belt, str(area), (x, y), 1, 1, (0, 255, 0))
    
    
    cv2.imshow("Frame", frame)
    cv2.imshow("belt", belt)
    cv2.imshow("threshold", threshold)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
    elif key == ord("n"):
        relay.turn_on()
    elif key == ord("m"):
        relay.turn_off()
    
cap.release()
cv2.destroyAllWindows()