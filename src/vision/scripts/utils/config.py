import vision.cfg.laneConfig as laneConfig
import vision.cfg.binsConfig as binsConfig
import vision.cfg.pickupConfig as pickupConfig

botCamTopic = '/bot_camera/camera/image_raw'
frontCamTopic = '/front_camera/camera/image_raw'
botCamLocalTopic = '/bottomcam/camera/image_rect_color'
frontCamLocalTopic = ''

compassTopic = '/euler'
visionFilterTopic = '/Vision/image_filter'

screen = { 'width': 640, 'height': 480 }
