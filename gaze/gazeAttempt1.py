import cv2
import dlib


def shape_to_np(shape, dtype="int"):
    coords = np.zeros((68, 2), dtype=dtype)
    for i in range(0, 68):
        coords[i] = (shape.part(i).x, shape.part(i).y)
    return coords

predictor = dlib.shape_predictor('shape_68.dat')
for (i, rect) in enumerate(rects):
    shape = predictor(gray, rect)
    shape = shape_to_np(shape)
    for (x, y) in shape:
        cv2.circle(img, (x, y), 2, (0, 0, 255), -1)

# following this guide 
# https://towardsdatascience.com/real-time-eye-tracking-using-opencv-and-dlib-b504ca724ac6
# https://github.com/vardanagarwal/Proctoring-AI
def main(): 
    img = cv2.imread('image.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale detector = dlib.get_frontal_face_detector()
    rects = detector(gray, 1) # rects contains all the faces detected
  
if __name__=="__main__": 
    main() 