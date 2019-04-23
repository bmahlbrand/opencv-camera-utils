import numpy as np
import cv2

rawDumpFolder = 'raw'
imageDump = rawDumpFolder + '/' + 'screenshots' + '/'
videoDump = rawDumpFolder + '/' + 'stream' + '/'

# def stream(device):
#     cap = cv2.VideoCapture(device)

#     while(True):
#         ret, frame = cap.read()

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Display the resulting frame
#         cv2.imshow('frame', gray)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

def stream(device, callback = None):
    cap = cv2.VideoCapture(device)

    i = 0

    while(True):
        
        ret, frame = cap.read()

        if callback != None:
            frame = callback(frame)
        
        cv2.imshow('frame', frame)
        
        key = cv2.waitKey(1)
        
        if key & 0xFF == ord('s'):
            cv2.imwrite(imageDump + 'frame' + str(i) + '.png', frame)
        elif key & 0xFF == ord('q'):
            break
        
        i += 1

    cap.release()
    cv2.destroyAllWindows()

def playVideo(filename):
    cap = cv2.VideoCapture(filename)
    
    while(cap.isOpened()):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def saveStream(device, filename):
    cap = cv2.VideoCapture(device)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(filename, fourcc, 20.0, (640,480))
 
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            # frame = cv2.flip(frame,0)
            # frame = auto_canny(frame)
            
            # frame = cv2.Laplacian(frame,cv2.CV_8U, ksize=5)
            # sobelx = cv2.Sobel(frame,cv2.CV_8U,1,0,ksize=3)
            # sobely = cv2.Sobel(frame,cv2.CV_8U,0,1,ksize=3)
            # frame = sobelx + sobely
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            # write the flipped frame
            out.write(frame)
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    # Release everything if job is finished
    cap.release()
    out.release()
    cv2.destroyAllWindows()