# SPDX-License-Identifier: GPL-2.0-or-later
 
bl_info = {
    "name": "Select by camera frame",
    "description": "Select objects according to camera frame",
    "author": "Samuel Bernou, Swann Martinez",
    "version": (0, 3, 0),
    "blender": (5, 0, 0),
    "location": "View3D",
    "warning": "",
    "wiki_url": "https://github.com/Pullusb/selectByCamFrame",
    "category": "Object" }
    

# coding: utf-8
import bpy
from mathutils import Vector
from time import time
import numpy

IMAX = 90000000

# [object type, filter property, icon]
TYPELIST = [
    ['MESH', 'slcf_mesh', 'OUTLINER_OB_MESH'],
    ['CURVE', 'slcf_curve', 'OUTLINER_OB_CURVE'],
    ['ARMATURE', 'slcf_armature', 'OUTLINER_OB_ARMATURE'],
    ['LATTICE', 'slcf_lattice', 'OUTLINER_OB_LATTICE'],
    ['FONT', 'slcf_text', 'OUTLINER_OB_FONT'],
    ['EMPTY', 'slcf_empty', 'OUTLINER_OB_EMPTY'],
    ['CAMERA', 'slcf_camera', 'OUTLINER_OB_CAMERA'],
    ['LIGHT', 'slcf_lamp', 'OUTLINER_OB_LIGHT'],
    ['SURFACE', 'slcf_surface', 'OUTLINER_OB_SURFACE'],
    ['META', 'slcf_metaball', 'OUTLINER_OB_META'],
    ['SPEAKER', 'slcf_speaker', 'OUTLINER_OB_SPEAKER'],
    ['GREASEPENCIL', 'slcf_greasepencil', 'OUTLINER_OB_GREASEPENCIL'],
    ['CURVES', 'slcf_curves', 'OUTLINER_OB_CURVES'],
    ['POINTCLOUD', 'slcf_pointcloud', 'OUTLINER_OB_POINTCLOUD'],
    ['VOLUME', 'slcf_volume', 'OUTLINER_OB_VOLUME'],
    ['LIGHT_PROBE', 'slcf_lightprobe', 'OUTLINER_OB_LIGHTPROBE'],
    ]

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


def get_world_bbox(obj, matrix=None):
    mat = obj.matrix_world if matrix is None else matrix
    return [mat @ Vector(i) for i in obj.bound_box]


def get_instances_bboxes(depsgraph, instancers):
    '''
    Return a dict {instancer name_full: [world bbox of each instanced sub-object]}
    for instancers (collection instance, vertex/face instancing, particles, geometry nodes instances...)

    instancers : set of original objects name_full to gather instances for
    '''
    bboxes = {}
    for inst in depsgraph.object_instances:
        if not inst.is_instance:
            continue
        parent_name = inst.parent.original.name_full
        if parent_name not in instancers:
            continue
        # instance data is only valid during this iteration step
        bboxes.setdefault(parent_name, []).append(get_world_bbox(inst.object, inst.matrix_world.copy()))
    return bboxes


def sat_intersect(mm_cam, frustum_planes, obj_bb):
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
    :param margin: frustum external margin (safety zone), in fraction of frame size on each side
    :type margin: float
    """
    cam_data = cam.data

    # frame corners in camera local space (handle lens shift, sensor fit, aspect ratio and ortho)
    frame = cam_data.view_frame(scene=scn)
    center = sum(frame, Vector()) / 4

    # corners keyed by (x sign, y sign) around frame center
    corners = {}
    for co in frame:
        co = center + (co - center) * (1 + 2 * margin)
        corners[(co.x > center.x, co.y > center.y)] = co

    def at_depth(co, depth):
        if cam_data.type == 'ORTHO':
            return Vector((co.x, co.y, -depth))
        # perspective (panoramic cameras are treated as perspective)
        return co * (depth / -co.z)

    # box indices order expected by construct_frustum_planes
    layout = [
        ((False, False), cam_data.clip_end),
        ((False, False), cam_data.clip_start),
        ((False, True), cam_data.clip_start),
        ((False, True), cam_data.clip_end),
        ((True, False), cam_data.clip_end),
        ((True, False), cam_data.clip_start),
        ((True, True), cam_data.clip_start),
        ((True, True), cam_data.clip_end),
    ]
    box = [at_depth(corners[key], depth) for key, depth in layout]

    return [cam.matrix_world @ co for co in box]


def construct_frustum_planes(cf):
    return [construct_plane(cf[0], cf[2], cf[3]),
            construct_plane(cf[3], cf[2], cf[7]),
            construct_plane(cf[7], cf[6], cf[4]),
            construct_plane(cf[5], cf[0], cf[4]),
            construct_plane(cf[4], cf[0], cf[7]),
            construct_plane(cf[2], cf[1], cf[5])]


class CAM_PG_hide_store_item(bpy.types.PropertyGroup):
    ob : bpy.props.PointerProperty(type=bpy.types.Object)
    hidden : bpy.props.BoolProperty()


class CAM_PG_select_cam_frame_props(bpy.types.PropertyGroup):
    slcf_anim : bpy.props.BoolProperty(
            name="Animation",
            description="Selection takes all frame of scene frame range into account\n(long operation if lots of frames/objects)",
            default=False)

    slcf_additive_select : bpy.props.BoolProperty(
            name="Additive Selection",
            description="Add to current selection. Else select/deselect everything",
            default=False)
    slcf_margin : bpy.props.FloatProperty(
            name="Margin",
            description="Use a margin around framing (inside if negative value)\
                \nA little margin can be a safety to avoid having an object being wrongly evaluated as outside the frame\
                \n(part of the object can be inside with bounding_box corner all outside)\
                \ndefault=0.03",
            default=0.03, min=-0.49, max=0.5, soft_min=0, soft_max=0.2, step=0.01, precision=3, unit='NONE')

    slcf_filter : bpy.props.BoolProperty(name='Object Filter', default=False)

    slcf_mesh : bpy.props.BoolProperty(name='mesh', default=True)
    slcf_curve : bpy.props.BoolProperty(name='curve', default=True)
    slcf_surface : bpy.props.BoolProperty(name='surface', default=True)
    slcf_metaball : bpy.props.BoolProperty(name='metaball', default=True)
    slcf_text : bpy.props.BoolProperty(name='text', default=True)
    slcf_armature : bpy.props.BoolProperty(name='armature', default=True)
    slcf_lattice : bpy.props.BoolProperty(name='lattice', default=True)
    slcf_empty : bpy.props.BoolProperty(name='empty', default=True)
    slcf_speaker : bpy.props.BoolProperty(name='speaker', default=True)
    slcf_camera : bpy.props.BoolProperty(name='camera', default=True)
    slcf_lamp : bpy.props.BoolProperty(name='lamp', default=True)
    slcf_greasepencil : bpy.props.BoolProperty(name='grease pencil', default=True)
    slcf_curves : bpy.props.BoolProperty(name='hair curves', default=True)
    slcf_pointcloud : bpy.props.BoolProperty(name='point cloud', default=True)
    slcf_volume : bpy.props.BoolProperty(name='volume', default=True)
    slcf_lightprobe : bpy.props.BoolProperty(name='light probe', default=True)

    # objects and their hide state before toggle, used to restore
    slcf_render_store : bpy.props.CollectionProperty(type=CAM_PG_hide_store_item)
    slcf_viewport_store : bpy.props.CollectionProperty(type=CAM_PG_hide_store_item)


def frame_selection(outside=True, anim=False, add=False, margin=0.03, ob_filter=None):
    '''
    Set selection according to camera frame (everything except camera).
    Args:
    outside : if true Select objects if there outside camera frame (bounding box). Else inside frame
    anim : if True, check for every frame in time range. Else use only current frame.
    add : if True, add to current selection. Else select/deselect everything
    margin : have a (safety) margin outside framing (or inside if negative value).
    ob_filter : a list or a tuple to restrict object type (if None, all type)
    - all type : see TYPELIST (bpy.types.Object.type identifiers)
    '''

    scene = bpy.context.scene
    view_layer = bpy.context.view_layer
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
    
    # only objects in the view layer can be selected (scene.objects includes excluded collections)
    selectables = [o for o in view_layer.objects if o.visible_get(view_layer=view_layer) and not o.hide_select]

    if ob_filter:
        base = [o for o in selectables if o.type in ob_filter]
        if not add:
            #deselect unwanted type since there won't be treated #can deselect everythin as well...
            for o in selectables:
                if o.type not in ob_filter:
                    o.select_set(False)
    else:#all
        base = selectables

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
            mm_cam.append(get_bb_min_max_on_axe(plane[:3], cam_frustum))

        # bbox of instancer is reduced to its own geometry (a point for empties), check instanced sub-objects too
        depsgraph = bpy.context.evaluated_depsgraph_get()
        instances_bboxes = get_instances_bboxes(depsgraph, {o.name_full for o in pool})

        for ob_id, o in enumerate(pool):
            bboxes = [get_world_bbox(o)] + instances_bboxes.get(o.name_full, [])
            if any(sat_intersect(mm_cam, cam_planes, bb) for bb in bboxes):
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
                    o.select_set(True)
            else:
                #set selection for each object
                o.select_set(o not in visibles)
    
    else:
        #select inside frame.
        for o in base:
            if add:
                if o in visibles:
                    #add inner object to selection
                    o.select_set(True)
            else:
                #set selection for each object
                o.select_set(o in visibles)
    
    #reset
    if anim:
        scene.frame_set(cur)
        wm.progress_end()#progress-OSD
    if scene.camera.name in view_layer.objects:
        scene.camera.select_set(False)

    return


class SELECT_OT_by_cam_frame(bpy.types.Operator):
    bl_idname = "select.by_cam_frame"
    bl_label = "Select By Cam Frame"
    bl_description = "Set selection according to camera frame"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT' and context.scene.camera is not None

    #duplicate margin_adjust as a self.prop so it can be adjusted in the redo
    margin_adjust : bpy.props.FloatProperty(
        name="margin adjust",
        description="adjust margin around framing",
        default=0.03, min=-0.49, max=0.5, soft_min=-0.4, soft_max=0.5, step=0.01, precision=3, unit='NONE')

    outside_frame : bpy.props.BoolProperty()

    def execute(self, context):
        ob_filter = []
        if context.scene.camf_sel.slcf_filter:
            for p in TYPELIST:
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


HIDE_MODES = [
    ('RENDER', 'Render', 'Object render visibility (hide_render)'),
    ('VIEWPORT', 'Viewport', 'Object viewport visibility (hide_viewport)'),
    ]


def get_hide_store(context, mode):
    props = context.scene.camf_sel
    return props.slcf_render_store if mode == 'RENDER' else props.slcf_viewport_store


class SELECT_OT_cam_frame_toggle_hide(bpy.types.Operator):
    bl_idname = "select.cam_frame_toggle_hide"
    bl_label = "Toggle Selection Visibility"
    bl_description = "Toggle visibility of selected objects\
        \nHide all if any is visible, else show all\
        \nInitial state of objects is stored to be restored later"
    bl_options = {"REGISTER", "UNDO"}

    mode : bpy.props.EnumProperty(items=HIDE_MODES, default='RENDER')

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT' and bool(context.selected_objects)

    def execute(self, context):
        attr = 'hide_render' if self.mode == 'RENDER' else 'hide_viewport'
        store = get_hide_store(context, self.mode)
        selection = context.selected_objects

        # store only objects not already stored, to keep their original state
        stored = {item.ob for item in store if item.ob}
        for o in selection:
            if o not in stored:
                item = store.add()
                item.ob = o
                item.hidden = getattr(o, attr)

        ## Hide if at least one is visible, Unhide if all hidden (valid only for hide_render)
        hide = not all(getattr(o, attr) for o in selection)
        for o in selection:
            setattr(o, attr, hide)

        self.report({'INFO'}, f"{'Hide' if hide else 'Show'} {self.mode.lower()}: {len(selection)} object(s)")
        return {"FINISHED"}


class SELECT_OT_cam_frame_restore_hide(bpy.types.Operator):
    bl_idname = "select.cam_frame_restore_hide"
    bl_label = "Restore Visibility"
    bl_description = "Restore visibility of objects stored before toggle, and select them back"
    bl_options = {"REGISTER", "UNDO"}

    mode : bpy.props.EnumProperty(items=HIDE_MODES, default='RENDER')

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def execute(self, context):
        attr = 'hide_render' if self.mode == 'RENDER' else 'hide_viewport'
        store = get_hide_store(context, self.mode)
        view_layer = context.view_layer

        obs = [item.ob for item in store if item.ob]
        for item in store:
            if item.ob:
                setattr(item.ob, attr, item.hidden)
        store.clear()

        # set visibility first, hidden objects can't be selected
        for o in obs:
            if o.name in view_layer.objects and o.visible_get(view_layer=view_layer) and not o.hide_select:
                o.select_set(True)

        self.report({'INFO'}, f"Restored {self.mode.lower()} visibility: {len(obs)} object(s)")
        return {"FINISHED"}

  
class SELECT_PT_by_cam_frame(bpy.types.Panel):
    bl_idname = "SELECT_PT_by_cam_frame"
    bl_label = "Camera Frame Selection"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Tool"

    def draw(self, context):
        props = context.scene.camf_sel
        layout = self.layout
        col = layout.column()
        #options
        row = col.row(align=True)
        row.prop(props, "slcf_anim")
        row.prop(props, "slcf_additive_select", text='Additive Select')
        
        #margin slider
        col.prop(props, "slcf_margin")

        #filters
        box = layout.box()
        row = box.row()
        row.prop(props, 'slcf_filter', icon="FILTER")#icon_only=True#icon_tria(props.slcf_filter)
        if props.slcf_filter:
            row = box.row(align=True)
            for obspec in TYPELIST:
                row.prop(props, obspec[1], icon=obspec[2], icon_only=True)

        #launch buttons
        row=layout.row(align=True)
        row.operator('select.by_cam_frame',text="Select Inside Cam").outside_frame = False 
        row.operator('select.by_cam_frame',text="Select Outside Cam").outside_frame = True

        #visibility toggles, with restore button when a state is stored
        col = layout.column(align=True)
        col.label(text='Selection Visibility Management:')
        for mode, text, icon in (
                ('RENDER', "Toggle render", 'RESTRICT_RENDER_OFF'),
                ('VIEWPORT', "Toggle viewport", 'RESTRICT_VIEW_OFF'),
                ):
            row = col.row(align=True)
            row.operator('select.cam_frame_toggle_hide', text=text, icon=icon).mode = mode
            if len(get_hide_store(context, mode)):
                row.operator('select.cam_frame_restore_hide', text="", icon='LOOP_BACK').mode = mode


### --- REGISTER

classes = (
CAM_PG_hide_store_item,
CAM_PG_select_cam_frame_props,
SELECT_OT_by_cam_frame,
SELECT_OT_cam_frame_toggle_hide,
SELECT_OT_cam_frame_restore_hide,
SELECT_PT_by_cam_frame,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.camf_sel = bpy.props.PointerProperty(type=CAM_PG_select_cam_frame_props)

    

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.camf_sel


if __name__ == "__main__":
    register()
