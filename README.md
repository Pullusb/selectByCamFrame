# Select by camera frame

Blender addon - Select objects according to camera frame

**[Download latest (2.8 / 3.x+)](https://github.com/Pullusb/selectByCamFrame/archive/refs/heads/master.zip)**

For older blender 2.7 version go [here](https://github.com/Pullusb/SB_blender_addons_old_2_7)

### Description:

Select all objects that are in (or out) camera framing.
Usefull to hide/delete objects outside of view to gain viewport performance or simply clean scene.

Works by checking bounding box of objects.
To launch, use either "Select inside cam" or "Select outside cam" (Only available for object mode)

### Panel and demo:
![panel](https://github.com/Pullusb/images_repo/blob/master/Bl_selectCameraFrame_frustum.png)

**options:**

- Anim - Check for every frame in time range. Else use only current frame.
- Additive select - Add to current selection. Else select/deselect everything
- Margin - Extend a selection margin outside framing (or inside if negative)
note: Margin default value is 0.03 as safety to avoid having an object being wrongly evaluated as outside the frame (part of the object can be visible inside with bounding_box corner all outside).
 
- filter - A list of object type to restrict selection only to those type
