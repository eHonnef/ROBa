from Streams import RGBimage, DepthImage, kill_openni
import keyboard
import cv2

img = RGBimage()
dep = DepthImage()

print("Press ESC to terminate!")

while True:
  if (cv2.waitKey(1) % 0x100) == 27 or keyboard.is_pressed("esc"):
    break

  cv2.imshow("rgb", img.get_rgb_img())
  cv2.imshow("dep", dep.get_depth_img()[1])

cv2.destroyAllWindows()
img.stop_stream()
kill_openni()
print("TCHAU!")
