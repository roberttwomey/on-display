import cv2

# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# def testDevice(source):
#    cap = cv2.VideoCapture(source) 
#    if cap is None or not cap.isOpened():
#        print('Warning: unable to open video source: ', source)

# testDevice(0)


# To capture video from webcam. 
cap = cv2.VideoCapture(2) # last device

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# To use a video file as input 
# cap = cv2.VideoCapture('filename.mp4')

while True:
    # Read the frame
    _, img = cap.read()
    if img is not None:
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(150,150))
        # Draw the rectangle around each face
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        # Display
        cv2.imshow('img', img)
        
    # Stop if escape key is pressed    
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break
# Release the VideoCapture object
cap.release()