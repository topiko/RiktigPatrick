from dm_control import mjcf
from utils import display_video
import PIL.Image
import numpy as np
import matplotlib.pyplot as plt


class RPHead(object):

    def __init__(self, head_H, rgba):
        self.rgba = rgba
        self.model = mjcf.RootElement("head")

        # Defaults:
        self.model.default.joint.damping = 1
        self.model.default.joint.type = 'hinge'

        head_D=.2
        head_W=.1
        reye = head_W/2


        # Headhead_H:
        head = self.model.worldbody.add('body', name='headbody')

        # Joints
        neck = head.add('joint', name='necknodd', axis=[0, 1, 0])
        eyelr = head.add('joint', name='neckturn', axis=[0, 0, 1])

        head.add('geom',
                 type='box',
                 name='headgeom',
                 mass=.2,
                 size=[head_D/2, head_W/2, head_H/2],
                 pos = [.0,.0, head_H/2],
                 rgba=self.rgba)

        # Eye:
        eye = head.add('body',
                       name='eye',
                       pos=[0, 0, head_H/2])
        eye.add('geom', type="cylinder",
                name="eyecyl",
                mass=.1,
                fromto=[0, 0, 0, head_D, 0, 0],
                size=[reye])

        # Position actuators:
        kp = 10
        self.model.actuator.add('position', name='necknodd', joint=neck, kp=kp)
        self.model.actuator.add('position', name='neckturn', joint=eyelr, kp=kp)


class RPFrame(object):

    def __init__(self, height, rgba):
        self.model = mjcf.RootElement("frame")

        body_D = .1
        body_W = .2
        body_H = height
        r_wheel=height/2

        frame = self.model.worldbody.add('body', name='framebody')
        frame.add('geom',
                  name='framebox',
                  type='box',
                  mass=1,
                  size=[body_D/2,body_W/2,body_H/2],
                  pos=[0,0,body_H/2],
                  rgba=rgba)



        for diry, key in zip([-1, 1], ['rightwheel', 'leftwheel']):
            y = diry * (body_W/2 + .001)
            wheel = frame.add('body',
                           name=key,
                           pos=[0, y, 0])
            wheel.add('geom',
                   type="cylinder",
                   name=key + "cyl",
                   fromto=[0, 0, 0, 0, diry*.02, 0],
                   friction=(2, 0.005, 0.0001),
                   size=[r_wheel])

            wheel = wheel.add('joint', name=key, axis=[0, 1, 0])
            self.model.actuator.add('motor',
                                    name='actuator_' + key,
                                    joint=wheel,
                                    gear=(1,),
                                    )



class SimRP(object):

    def __init__(self):
        rgba =[0.70269912, 0.4307427,  0.33218761, 1.]
        rphead = RPHead(HEADH, rgba).model
        rpbody = RPFrame(BODYH, rgba).model

        head_site = rpbody.worldbody.add('site', name='headsite', pos=[0,0,BODYH])
        head_site.attach(rphead)

        self.model = rpbody


def make_ball():
    model = mjcf.RootElement(model='ball')

    # Make the torso geom.
    model.worldbody.add('geom',
                        name='ball',
                        type='sphere',
                        size=[.1]) #, rgba=rgba)
    return model



def make_arena():
    arena = mjcf.RootElement("arena")
    chequered = arena.asset.add('texture', type='2d', builtin='checker', width=400,
                                height=400, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
    grid = arena.asset.add('material', name='grid', texture=chequered,
                           texrepeat=[10, 10], reflectance=.2)
    arena.worldbody.add('geom', type='plane', size=[3, 3, .1], material=grid)
    for x in [-2, 2]:
        arena.worldbody.add('light', name='light_{}'.format(x), pos=[x, -1, 3], dir=[-x, 1, -2])

    #TODO: use quat for better camera angle
    camera_site = arena.worldbody.add('site', name='camerasite', pos=[.1,-4,3], euler=[60,0,0])
    camera = mjcf.RootElement('camera')
    camera.worldbody.add('camera', name='camera', mode="trackcom")
    camera_site.attach(camera)

    return arena



# Instantiate creature.

BODYH = .20
HEADH = BODYH
simrp = SimRP().model

arena = make_arena()
xpos, ypos, zpos = [0,0, BODYH*1]
spawn_site = arena.worldbody.add('site', pos=[xpos, ypos, zpos], group=3)
#ball_site = arena.worldbody.add('site', pos=[1, ypos, 2], group=3)

spawn_site.attach(simrp).add('freejoint')

#ball = make_ball()
#ball_site.attach(ball).add('freejoint')

# Instantiate the physics and render.
physics = mjcf.Physics.from_mjcf_model(arena)
img = PIL.Image.fromarray(physics.render(camera_id=0, height=400, width=500))
plt.subplots(figsize=(10,8))
plt.imshow(img)
plt.axis('off')
plt.show()

duration = 10   # (Seconds)
framerate = 30
video = []
pos_x = []
pos_y = []
#torso = rpbody.find('geom', 'frame/framebox')

necknodd = simrp.find('actuator', 'head/necknodd') #necknodd
neckturn = simrp.find('actuator', 'head/neckturn') #necknodd


#print(rpbody.to_xml_string())
print(arena.to_xml_string())
fname = '/home/topiko/.mujoco/mujoco210/model/rp.xml'
with open(fname, "w") as f:
    f.write(arena.to_xml_string())

# Control signal frequency, phase, amplitude.
freq = 5
amp = 0.2

# Simulate, saving video frames and torso locations.
physics.reset()
while physics.data.time < duration:
    # Inject controls and step the physics.
    physics.bind(necknodd).ctrl = amp * np.sin(freq * physics.data.time + np.pi/2)
    physics.bind(neckturn).ctrl = amp * np.sin(freq * physics.data.time)
    physics.step()

    # Save torso horizontal positions using bind().
    #pos_x.append(physics.bind(torso).xpos[0].copy())
    #pos_y.append(physics.bind(torso).xpos[1].copy())

    # Save video frames.
    if len(video) < physics.data.time * framerate:
        pixels = physics.render(camera_id=0) #scene_option='follow')
        video.append(pixels.copy())

display_video(video, framerate)
