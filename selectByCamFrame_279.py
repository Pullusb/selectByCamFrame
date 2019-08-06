bl_info = {
    "name": "Select by camera frame",
    "description": "Select objects according to camera frame",
    "author": "Samuel Bernou, Swann Martinez",
    "version": (0, 0, 3),
    "blender": (2, 79, 0),
    "location": "View3D",
    "warning": "",
    "wiki_url": "https://github.com/Pullusb/selectByCamFrame",
    "category": "Object" }
    

# coding: utf-8
import bpy
from mathutils import Vector
from time import time
from bpy_extras.object_utils import world_to_camera_view
import numpy
import math

IMAX = 90000000

def normalize(v):
    norm=numpy.linalg.norm(v, ord=1)
    if norm==0:
        norm=numpy.finfo(v.dtype).eps
    return v/norm

def get_bb_min_max_on_axe(axis, bb):
    bb_min = IMAX
    bb_max = -IMAX

    for vertice in bb:
        projection = numpy.dot(vertice,axis)
        bb_min = min(bb_min,projection)
        bb_max = max(bb_max,projection)

    return [bb_min,bb_max]


def sat_intersect(mm_cam, frustum_planes, obj):
    obj_bb = [obj.matrix_world * Vector(i) for i in obj.bound_box]
    for i,plane in enumerate(frustum_planes):
        mm_obj = get_bb_min_max_on_axe(plane[:3],obj_bb)

        if not overlap(mm_obj[0],mm_obj[1],mm_cam[i][0],mm_cam[i][1]):
            return False
    
    return True 


def overlap(min1, max1, min2, max2):
    return is_between(min2, min1, max1) or is_between(min1, min2, max2)


def is_between(val, lower_bound, upper_bound):
    return lower_bound <= val and val <= upper_bound


def construct_plane(p1, p2, p3, origin=None):
    """
    Construct a plane from 3 points

    :param p1: point a
    :type p1: array [x,y,z]
    :param p2: point b
    :type p2: array [x,y,z]
    :param p3: point c
    :type p3: array [x,y,z]
    :param origin: point used to orient plane normal (optional)
    :type origin: array [x,y,z]
    """
    v1 = p3 - p1
    v2 = p2 - p1
    cp = normalize(numpy.cross(v1, v2))
    d = numpy.dot(cp, p3)
    
    if origin:
        if point_plane_distance([cp[0], cp[1], cp[2], d], origin) < 0:
            return [-cp[0], -cp[1], -cp[2], -d]

    return [cp[0], cp[1], cp[2], d]


def point_plane_distance(plane, point):
    return numpy.dot(plane[:3], (point))-plane[3]


def construct_frustum_bb(cam, scn, margin=0.03):
    """
    Construct the camera frustum as a box  

    :param cam: source camera for frustum computation 
    :type cam: bpy.types.Camera
    :param scn: source scene for frustum computation 
    :type scn: bpy.type.Scene
    :param margin: frustum external margin (safety zone)  
    :type margin: float
    """
    cam_data = cam.data
    box = [[0, 0, 0] for i in range(8)]

    aspx = scn.render.resolution_x * scn.render.pixel_aspect_x
    aspy = scn.render.resolution_y * scn.render.pixel_aspect_y

    ratiox = min(aspx / aspy, 1.0)
    ratioy = min(aspy / aspx, 1.0)

    angleofview = 2.0 * \
        math.atan(cam_data.sensor_width / (2.0 * cam_data.lens))
    oppositeclipsta = math.tan(angleofview / 2.0) * cam_data.clip_start
    oppositeclipend = math.tan(angleofview / 2.0) * cam_data.clip_end

    box[2][0] = box[1][0] = -oppositeclipsta * ratiox
    box[0][0] = box[3][0] = -oppositeclipend * ratiox
    box[5][0] = box[6][0] = +oppositeclipsta * ratiox
    box[4][0] = box[7][0] = +oppositeclipend * ratiox
    box[1][1] = box[5][1] = -oppositeclipsta * ratioy
    box[0][1] = box[4][1] = -oppositeclipend * ratioy
    box[2][1] = box[6][1] = +oppositeclipsta * ratioy
    box[3][1] = box[7][1] = +oppositeclipend * ratioy
    box[0][2] = box[3][2] = box[4][2] = box[7][2] = -cam_data.clip_end
    box[1][2] = box[2][2] = box[5][2] = box[6][2] = -cam_data.clip_start

    return [cam.matrix_world * Vector(i) for i in box]


def construct_frustum_planes(cf):
    return [construct_plane(cf[0], cf[2], cf[3]),
            construct_plane(cf[3], cf[2], cf[7]),
            construct_plane(cf[7], cf[6], cf[4]),
            construct_plane(cf[5], cf[0], cf[4]),
            construct_plane(cf[4], cf[0], cf[7]),
            construct_plane(cf[2], cf[1], cf[5])]


class CAM_PROPS_selectCamFrameProps(bpy.types.PropertyGroup):
    slcf_anim = bpy.props.BoolProperty(
            name="animation",
            description="Selection takes all frame of scene frame range into account\n(long operation if lots of frames/objects)",
            default=False)

    slcf_additive_select = bpy.props.BoolProperty(
            name="additive selection",
            description="Add to current selection. Else select/deselect everything",
            default=False)
    slcf_margin = bpy.props.FloatProperty(
            name="margin",
            description="Use a margin around framing (inside if negative value)\nA little margin can be a safety to avoid having an object being wrongly evaluated as outside the frame\n(part of the object can be inside with bounding_box corner all outside)\ndefault=0.03, min=-0.49, max=0.5",
            default=0.03, min=-0.49, max=0.5, soft_min=0, soft_max=0.2, step=0.01, precision=3, unit='NONE')

    slcf_filter = bpy.props.BoolProperty(name='Object filter', default=False)

    slcf_mesh = bpy.props.BoolProperty(name='mesh', default=True)
    slcf_curve = bpy.props.BoolProperty(name='curve', default=True)
    slcf_surface = bpy.props.BoolProperty(name='surface', default=True)
    slcf_metaball = bpy.props.BoolProperty(name='metaball', default=True)
    slcf_text = bpy.props.BoolProperty(name='text', default=True)
    slcf_armature = bpy.props.BoolProperty(name='armature', default=True)
    slcf_lattice = bpy.props.BoolProperty(name='lattice', default=True)
    slcf_empty = bpy.props.BoolProperty(name='empty', default=True)
    slcf_speaker = bpy.props.BoolProperty(name='speaker', default=True)
    slcf_camera = bpy.props.BoolProperty(name='camera', default=True)
    slcf_lamp = bpy.props.BoolProperty(name='lamp', default=True)


def frame_selection(outside=True, anim=False, add=False, margin=0.03, ob_filter=None):
    '''
    Set selection according to camera frame (everything except camera).
    Args:
    outside : if true Select objects if there outside camera frame (bounding box). Else inside frame
    anim : if True, check for every frame in time range. Else use only current frame.
    add : if True, add to current selection. Else select/deselect everything
    margin : have a (safety) margin outside framing (or inside if negative value).
    ob_filter : a list or a tuple to restrict object type (if None, all type)
    - all type : ('MESH', 'CURVE', 'SURFACE', 'META', 'TEXT', 'ARMATURE', 'LATTICE', 'EMPTY', 'SPEAKER', 'CAMERA', 'LAMP',)
    '''

    scene = bpy.context.scene
    cur = scene.frame_current
    if anim:
        start = scene.frame_start
        end = scene.frame_end + 1
        wm = bpy.context.window_manager#progress-OSD
        wm.progress_begin(start, end)#progress-OSD

    else:
        #python range return only on current frame
        start = cur
        end = cur + 1
    
    if ob_filter:
        base = [o for o in scene.objects if o.type in ob_filter]
        if not add:
            #deselect unwanted type since there won't be treated #can deselect everythin as well...
            for o in scene.objects:
                if o.type not in ob_filter:
                    o.select = False
    else:#all
        base = scene.objects[:]#[o for o in scene.objects]

    pool = base.copy()
    visibles = []

    for i in range(start,end):
        if anim:
            scene.frame_set(i)
            wm.progress_update(i)#progress-OSD
        
        indexes = []
        
        cam_frustum = construct_frustum_bb(
            scene.camera,
            scene,
            margin)
        cam_planes = construct_frustum_planes(cam_frustum)

        #Precompute cam min max per plane normal
        mm_cam = []
        for plane in cam_planes:
            camera_min_max = get_bb_min_max_on_axe(plane[:3], cam_frustum)
            camera_min_max[0] -= margin
            camera_min_max[0] += margin
            mm_cam.append(camera_min_max)

        for ob_id, o in enumerate(pool):
            if sat_intersect(mm_cam, cam_planes, o):
                #get obj out of base list and go to visible list
                visibles.append(pool[ob_id])
                indexes.append(ob_id)
        
        if anim:
            #pop already seen objects to avoid recheck these
            pool=[o for i, o in enumerate(pool) if i not in indexes]
     
    if outside:
         #select outside frame (based on initial filter)
        for o in base:
            if add:
                if o not in visibles:
                    #add outer object to selection
                    o.select = True
            else:
                #set selection for each object
                o.select = o not in visibles
    
    else:
        #select inside frame.
        for o in base:
            if add:
                if o in visibles:
                    #add inner object to selection
                    o.select = True
            else:
                #set selection for each object
                o.select = o in visibles
    
    #reset
    if anim:
        scene.frame_set(cur)
        wm.progress_end()#progress-OSD
    scene.camera.select = False

    return


class SELECT_OT_by_cam_frame(bpy.types.Operator):
    bl_idname = "select.by_cam_frame"
    bl_label = "Select by cam frame"
    bl_description = "Set selection according to camera frame"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    #duplicate margin_adjust as a self.prop so it can be adjusted in the redo
    margin_adjust = bpy.props.FloatProperty(
        name="margin adjust",
        description="adjust margin around framing",
        default=0.03, min=-0.49, max=0.5, soft_min=-0.4, soft_max=0.5, step=0.01, precision=3, unit='NONE')

    outside_frame = bpy.props.BoolProperty()

    def execute(self, context):
        typelist = [
            ['MESH', 'slcf_mesh','OBJECT_DATA'],
            ['CURVE', 'slcf_curve','OUTLINER_OB_CURVE'],
            ['ARMATURE', 'slcf_armature','OUTLINER_OB_ARMATURE'],
            ['LATTICE', 'slcf_lattice','OUTLINER_OB_LATTICE'],
            ['TEXT', 'slcf_text','OUTLINER_OB_FONT'],
            ['EMPTY', 'slcf_empty','OUTLINER_OB_EMPTY'],
            ['CAMERA', 'slcf_camera','OUTLINER_OB_CAMERA'],
            ['LAMP', 'slcf_lamp','OUTLINER_OB_LAMP'],
            ['SURFACE', 'slcf_surface','OUTLINER_OB_SURFACE'],
            ['META', 'slcf_metaball','OUTLINER_OB_META'],
            ['SPEAKER', 'slcf_speaker','OUTLINER_OB_SPEAKER'],
            ]

        ob_filter = []
        if context.scene.camf_sel.slcf_filter:
            for p in typelist:
                if getattr(context.scene.camf_sel, p[1]):
                    ob_filter.append(p[0])

        start = time()
        frame_selection(outside=self.outside_frame,
            anim=context.scene.camf_sel.slcf_anim,
            add=context.scene.camf_sel.slcf_additive_select,
            margin=self.margin_adjust,
            ob_filter=ob_filter)
        
        exec_time = time() - start
        if exec_time > 1.0:print("frame selection time:", exec_time)

        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        #works only with self class internal properties, scene variable cannot be updated
        layout.prop(self, "margin_adjust")
    
    def invoke(self, context, event):
        #set internal variable margin_adjust same as margin panel propertie value (allow to update with the "redo" bl_option)
        self.margin_adjust = context.scene.camf_sel.slcf_margin
        return self.execute(context)

  
class SELECT_PT_by_cam_frame(bpy.types.Panel):
    bl_idname = "select_PT_by_cam_frame"
    bl_label = "Camera frame selection"
    bl_space_type = "VIEW_3D"
    bl_region_type = "TOOLS"
    bl_category = "Tools"

    def draw(self, context):
        props = context.scene.camf_sel
        layout = self.layout
        
        #options
        row=layout.row()
        row.prop(props, "slcf_anim")
        row.prop(props, "slcf_additive_select")
        
        #margin slider
        layout = self.layout
        layout.prop(props, "slcf_margin")

        #filters
        box = layout.box()
        row = box.row()
        row.prop(props, 'slcf_filter', icon="FILTER")#icon_only=True#icon_tria(props.slcf_filter)
        if props.slcf_filter:
            typelist = [
                ['MESH', 'slcf_mesh','OBJECT_DATA'],
                ['CURVE', 'slcf_curve','OUTLINER_OB_CURVE'],
                ['ARMATURE', 'slcf_armature','OUTLINER_OB_ARMATURE'],
                ['LATTICE', 'slcf_lattice','OUTLINER_OB_LATTICE'],
                ['TEXT', 'slcf_text','OUTLINER_OB_FONT'],
                ['EMPTY', 'slcf_empty','OUTLINER_OB_EMPTY'],
                ['CAMERA', 'slcf_camera','OUTLINER_OB_CAMERA'],
                ['LAMP', 'slcf_lamp','OUTLINER_OB_LAMP'],
                ['SURFACE', 'slcf_surface','OUTLINER_OB_SURFACE'],
                ['META', 'slcf_metaball','OUTLINER_OB_META'],
                ['SPEAKER', 'slcf_speaker','OUTLINER_OB_SPEAKER'],
                ]
            row = box.row(align = True)
            for obspec in typelist:
                row.prop(props, obspec[1], icon=obspec[2], icon_only=True)

        #launch buttons
        row=layout.row(align=True)
        row.operator('select.by_cam_frame',text="Select inside cam").outside_frame = False 
        row.operator('select.by_cam_frame',text="Select outside cam").outside_frame = True 



### --- REGISTER

classes = (
SELECT_OT_by_cam_frame,
SELECT_PT_by_cam_frame,
CAM_PROPS_selectCamFrameProps,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.camf_sel = bpy.props.PointerProperty(type=CAM_PROPS_selectCamFrameProps)

    

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.camf_sel


if __name__ == "__main__":
    register()
