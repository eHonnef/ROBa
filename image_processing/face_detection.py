from Streams import RGBimage
import keyboard
import cv2

# https://towardsdatascience.com/face-detection-in-2-minutes-using-opencv-python-90f89d7c0f81

# Load the cascade
face_cascade = cv2.CascadeClassifier('xml/haarcascade_frontalface_default.xml')

# start video stream
stream = RGBimage()

while True:
  if (cv2.waitKey(1) % 0x100) == 27 or keyboard.is_pressed("esc"):
    break

  # Read the input image
  img = stream.get_rgb_img()

  # Convert into grayscale
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  # Detect faces
  faces = face_cascade.detectMultiScale(gray, 1.1, 4)

  # Draw rectangle around the faces
  for (x, y, w, h) in faces:
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

  # Display the output
  cv2.imshow('img', img)

cv2.destroyAllWindows()
stream.stop_stream()
print("TCHAU!")
