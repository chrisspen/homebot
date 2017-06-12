#!/usr/bin/env python
"""
https://realpython.com/blog/python/face-detection-in-python-using-a-webcam/
"""

import time
import libzbar as zb
import cv2
from PIL import Image as PilImage

video_capture = cv2.VideoCapture(0)

t0 = time.time()
frames = 0
while True:
    frames += 1

    # Capture frame-by-frame
    ret, frame_np_ary = video_capture.read()

#     print type(frame_np_ary)

    gray = cv2.cvtColor(frame_np_ary, cv2.COLOR_BGR2GRAY)

#     faces = faceCascade.detectMultiScale(
#         gray,
#         scaleFactor=1.1,
#         minNeighbors=5,
#         minSize=(30, 30),
#         flags=cv2.cv.CV_HAAR_SCALE_IMAGE
#     )

    # Convert CV image to PIL Image.
    pil_img = PilImage.fromarray(gray)

    matches = zb.Image.from_im(pil_img).scan()
#     print matches

    if matches:
        for match in matches:
            # Draw a rectangle around the faces

            tl, bl, br, tr = match.locator
            x, y = bl
            w = abs(tl[0] - bl[0])
            h = abs(tl[1] - br[1])

            #cv2.rectangle(frame_np_ary, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.line(frame_np_ary, tl, bl, (0,255,0), 3)
            cv2.line(frame_np_ary, bl, br, (0,255,0), 3)
            cv2.line(frame_np_ary, br, tr, (0,255,0), 3)
            cv2.line(frame_np_ary, tr, tl, (0,255,0), 3)

    # Display the resulting frame_np_ary
    cv2.imshow('Video', frame_np_ary)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    fps = frames/float(time.time() - t0)
    if not frames % 10:
        print 'fps:', fps

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
