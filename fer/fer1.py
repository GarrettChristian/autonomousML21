import cv2
import sys
from keras.models import load_model
import numpy as np
from keras.preprocessing.image import img_to_array
from keras.preprocessing import image

# following
# https://realpython.com/face-detection-in-python-using-a-webcam/
# and https://medium.com/swlh/emotion-detection-using-opencv-and-keras-771260bbd7f7
def main(): 
    
    # cascPath = sys.argv[0]
    # faceCascade = cv2.CascadeClassifier(cascPath)
    faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    print("here")

    # classifier = load_model('EmotionDetectionModel.h5')
    classifier = load_model('/Users/garrettchristian/DocumentsDesktop/csSeniorYear/autonomousVehicles/autonomousMLgc/fer/EmotionDetectionModel.h5')
    class_labels=['Angry','Happy','Neutral','Sad','Surprise']
    print("here1")

    # 0 is your webcam you could also provide a file name for a video
    video_capture = cv2.VideoCapture(0)
    print("here2")

    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            # flags=cv2.cv.CV_HAAR_SCALE_IMAGE
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            roi_gray=gray[y:y+h,x:x+w]
            roi_gray=cv2.resize(roi_gray,(48,48),interpolation=cv2.INTER_AREA)

            if np.sum([roi_gray])!=0:
                roi=roi_gray.astype('float')/255.0
                roi=img_to_array(roi)
                roi=np.expand_dims(roi,axis=0)

                preds=classifier.predict(roi)[0]
                label=class_labels[preds.argmax()]
                label_position=(x,y)
                cv2.putText(frame,label,label_position,cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)
            else:
                cv2.putText(frame,'No Face Found',(20,20),cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)

        # Display the resulting frame
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()
  
if __name__=="__main__": 
    main() 