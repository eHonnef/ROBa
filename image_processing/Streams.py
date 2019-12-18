import numpy as np
from primesense import _openni2 as c_api
from primesense import openni2
import cv2

def kill_openni():
  openni2.unload()

class RGBimage:

  def __init__(self, width=320, height=240, fps=30):
    if not openni2.is_initialized():
      openni2.initialize("/usr/lib/")

    self.width = width
    self.height = height

    # maybe change this to a more specific device
    self.device = openni2.Device.open_any()
    self.rgb_stream = self.device.create_color_stream()
    self.rgb_stream.set_video_mode(
        c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
            resolutionX=width,
            resolutionY=height,
            fps=fps))
    self.rgb_stream.start()

  def get_rgb_img(self):
    bgr = np.fromstring(
        self.rgb_stream.read_frame().get_buffer_as_uint8(),
        dtype=np.uint8).reshape(self.height, self.width, 3)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb

  def stop_stream(self):
    self.rgb_stream.stop()
  

class DepthImage:

  def __init__(self, width=320, height=240, fps=30):
    if not openni2.is_initialized():
      openni2.initialize("/usr/lib/")

    self.width = width
    self.height = height

    # maybe change this to a more specific device
    self.device = openni2.Device.open_any()
    self.depth_stream = self.device.create_depth_stream()
    self.depth_stream.set_video_mode(
        c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
            resolutionX=width,
            resolutionY=height,
            fps=30))

    # self.depth_stream.set_mirroring_enabled(False)
    self.depth_stream.start()

  def get_depth_img(self):
    """
      Returns numpy ndarrays representing the raw and ranged depth images.
      Outputs:
          dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
          d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255    
      Note2:     
          .reshape(120,160) #smaller image for faster response 
                  OMAP/ARM default video configuration
          .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                  Requires .set_video_mode
      """
    dmap = np.frombuffer(
        self.depth_stream.read_frame().get_buffer_as_uint16(),
        dtype=np.uint16).reshape(self.height, self.width)

    d4d = np.uint8(dmap.astype(float) * 255 / 2**12 -
                   1)  # Correct the range. Depth images are 12bits
    d4d = cv2.cvtColor(d4d, cv2.COLOR_GRAY2RGB)

    # Shown unknowns in black
    d4d = 255 - d4d
    return dmap, d4d

  def stop_stream(self):
    self.depth_stream.stop()
