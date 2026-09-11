from dm_control import mjcf
from utils import display_video
import PIL.Image
import numpy as np
import matplotlib.pyplot as plt

class SimRP(object):

    """A 2-DoF leg with position actuators."""

    def __init__(self):
        self.rgba = [0.70269912, 0.4307427,  0.33218761, 1.]        # np.random.uniform([0, 0, 0, 1], [1, 1, 1, 1])
        self.model = mjcf.RootElement(model='RP')
        self.model.compiler.angle = 'radian'  # Use radians.

        camera_site = self.model.worldbody.add('site', name='camerasite', pos=[.5,-2,2], euler=[np.pi/4,0,0])
        camera = mjcf.RootElement('camera')
        camera.worldbody.add('camera', name='camera', mode="trackcom")
        camera_site.attach(camera)

        self.body_H = .1

        self.rp = self.build()

    def make_head(self):
        model = mjcf.RootElement("head")

        # Defaults:
        model.default.joint.damping = 100
        model.default.joint.type = 'hinge'


        length = .15
        reye = .05

        # Head:
        head = model.worldbody.add('body', name='headbody')

        # Joints
        neck = head.add('joint', name='necknodd', axis=[0, 1, 0])
        eyelr = head.add('joint', name='neckturn', axis=[0, 0, 1])

        head.add('geom',
                 type='box',
                 name='headgeom',
                 size=[0.1, 0.1, length],
                 pos = [0,0,length],
                 rgba=self.rgba)

        # Eye:
        eye = head.add('body',
                       name='eye',
                       pos=[0, 0, length])
        eye.add('geom', type="cylinder",
                name="eyecyl",
                fromto=[0, 0, 0, length, 0, 0],
                size=[reye])

        # Position actuators:
        kp = 100000
        model.actuator.add('position', name='necknodd', joint=neck, kp=kp)
        model.actuator.add('position', name='neckturn', joint=eyelr, kp=kp)

        return model

    def make_body(self):

        model = mjcf.RootElement("frame")


        # Make the torso geom.
        frame = model.worldbody.add('body', name='framebody')
        frame.add('geom',
                  name='framebox',
                  type='box',
                  mass=1,
                  size=[.3,.3,self.body_H],
                  pos=[0,0,self.body_H],
                  rgba=self.rgba)

        '''

        model.worldbody.add('geom',
                            name='framebox',
                            type='box',
                            size=[.3,.3,H], rgba=self.rgba)

        '''
        lw = frame.add('body',
                        name='leftwheel',
                        pos=[0, -.3 - .1, self.body_H])
        lw.add('geom', type="cylinder",
                name="lwcyl",
                fromto=[0, 0, 0, 0, .1, 0],
                size=[self.body_H*2])

        leftwheel = lw.add('joint', type='hinge', name='leftwheel', axis=[0, 1, 0])
        model.actuator.add('position', name='lwheel', joint=leftwheel)

        return model



    def build(self):
        Z = .2;
        body_site = self.model.worldbody.add('site', name='framepos', pos=[0,0,Z])
        body_site.attach(self.make_body())

        #TODO: how does these positions work.
        head_pos = [0,0,self.body_H*2+Z] #
        head_site = self.model.worldbody.add('site', name='headpos', pos=head_pos, euler=[0, 0, 0])
        head_site.attach(self.make_head())


        print(self.model.to_xml_string())
        return self.model

H = .20
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
    chequered = arena.asset.add('texture', type='2d', builtin='checker', width=300,
                                height=300, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
    grid = arena.asset.add('material', name='grid', texture=chequered,
                           texrepeat=[5, 5], reflectance=.2)
    arena.worldbody.add('geom', type='plane', size=[2, 2, .1], material=grid)
    for x in [-2, 2]:
        arena.worldbody.add('light', name='light_{}'.format(x), pos=[x, -1, 3], dir=[-x, 1, -2])

    return arena



# Instantiate creature.
creature = SimRP().rp #make_creature()

arena = make_arena()
xpos, ypos, zpos = [0,0,0]
spawn_site = arena.worldbody.add('site', pos=[xpos, ypos, zpos], group=3)
ball_site = arena.worldbody.add('site', pos=[1, ypos, 2], group=3)

spawn_site.attach(creature) #.add('freejoint')

ball = make_ball()
ball_site.attach(ball).add('freejoint')

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
torso = creature.find('geom', 'frame/framebox')

necknodd = creature.find('actuator', 'head/necknodd') #necknodd
neckturn = creature.find('actuator', 'head/neckturn') #necknodd

print(arena.to_xml_string())

# Control signal frequency, phase, amplitude.
freq = 5
amp = 0.4

# Simulate, saving video frames and torso locations.
physics.reset()
while physics.data.time < duration:
    # Inject controls and step the physics.
    physics.bind(necknodd).ctrl = amp * np.sin(freq * physics.data.time + np.pi/2)
    physics.bind(neckturn).ctrl = amp * np.sin(freq * physics.data.time)
    physics.step()

    # Save torso horizontal positions using bind().
    pos_x.append(physics.bind(torso).xpos[0].copy())
    pos_y.append(physics.bind(torso).xpos[1].copy())

    # Save video frames.
    if len(video) < physics.data.time * framerate:
        pixels = physics.render(camera_id=0) #scene_option='follow')
        video.append(pixels.copy())

display_video(video, framerate)
