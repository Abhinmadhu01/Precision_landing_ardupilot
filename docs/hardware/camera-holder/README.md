# IMX500 Top-Down Camera Holder

A 3D-printed holder that fixes the IMX500 camera so it looks straight down at the ground. It was designed in Onshape (free for non-commercial use) and printed in PLA.

TODO: add a photo of the printed holder as `images/holder_photo.jpg`, then add `![Printed camera holder](images/holder_photo.jpg)` here.

## Files

| File | Description |
| --- | --- |
| `cad/imx_camera_holder_top_down.stl` | Print-ready model. TODO: add file |
| `cad/imx_camera_holder_top_down.step` | Editable CAD export. TODO: add file |
| Onshape document | TODO: add public link |

Put the files in the `cad/` folder next to this README, and photos in `images/`.

## Print settings

| Setting | Value |
| --- | --- |
| Material | PLA |
| Printer | TODO |
| Layer height | TODO |
| Infill | TODO |
| Supports | TODO |
| Print orientation | TODO |

PLA softens in hot sun and inside a closed hot enclosure. If you fly in hot conditions, consider PETG.

## Hardware needed

| Item | Quantity |
| --- | --- |
| Screws and nuts for the camera | TODO (size and count) |
| Screws and nuts for mounting the holder to the frame | TODO (size and count) |
| Frame or arm it mounts to | TODO |

## Mounting

1. Attach the camera to the holder. Make sure the CSI cable has room and is not bent sharply.
2. Mount the holder to the frame so the lens points **straight down**. The camera is fixed (no gimbal).
3. Check the lens has a clear view and that no frame part, landing gear leg or propeller is in the picture.
4. Secure the cable so it cannot touch the propellers.

### Orientation (important)

The landing controller assumes:

- **Top of the image = drone nose (forward)**
- **Left of the image = drone left**

This follows from the control code: a tag appearing in the upper part of the image makes the drone move forward, and a tag on the left makes it move left. If your camera is rotated, either rotate the holder or change `sign_x` and `sign_y` in [../../../config/params.yaml](../../../config/params.yaml). Check this on the bench with propellers off: move a tag in front of the camera and confirm the commanded velocity points toward it.

## Checks before flight

- [ ] Holder is tight, no wobble when you shake the frame
- [ ] Image is sharp at the landing altitudes you will use
- [ ] Orientation checked as above
- [ ] Camera cable is secured away from propellers
