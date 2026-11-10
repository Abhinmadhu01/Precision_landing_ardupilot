# AprilTag Maximum Detection Range

How far away a tag of a given size can be detected, and how to choose tag sizes for the nested target.

## The equation

A camera sees an object of width `s` at distance `R` as a width of `p` pixels:

```text
p = s * f_px / R
```

A detector needs a minimum tag width `p_min` in pixels (including the black border). Solving for distance:

```text
R_max = s * f_px / p_min
```

where the focal length in pixels depends on the image width `W` and the horizontal field of view `HFOV`:

```text
f_px = (W / 2) / tan(HFOV / 2)
```

In GitHub math form:

$$R_{max} = \frac{s \cdot f_{px}}{p_{min}}, \qquad f_{px} = \frac{W/2}{\tan(\mathrm{HFOV}/2)}$$

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `s` | Printed tag edge length (outer edge of the black border) | m |
| `f_px` | Focal length in pixels | px |
| `W` | Image width of the stream you use | px |
| `HFOV` | Horizontal field of view of the camera | deg |
| `p_min` | Smallest tag width the detector still finds | px |

Consequences: range is proportional to tag size, and proportional to image width (through `f_px`), so doubling the resolution roughly doubles the range, in line with the field results.

## Worked example from field tests

Measured with the IMX500 and the 22.7 cm outer tag:

| Resolution | Measured max detection altitude |
| --- | --- |
| 640x480 | 7-8 m |
| 1280x720 | 10-12 m |

Assuming a horizontal field of view of about 66 degrees (the IMX500 camera module datasheet value, TODO: verify with your own camera calibration):

| Resolution | `f_px` | `p_min` implied by the measured range |
| --- | --- | --- |
| 640x480 | about 493 px | about 14-16 px |
| 1280x720 | about 986 px | about 19-22 px |

Calculation for 640x480 at 8 m: `p_min = 0.227 x 493 / 8 = 14 px`. At 1280x720 the implied `p_min` is higher than at 640x480, which is expected: detection range depends on more than pixel count (blur, lighting, tag contrast, detector settings), so use the measured value of your own setup.

## Predicted range for the nested tags

Using `p_min` of about 15 px at 640x480 and about 20 px at 1280x720 (your measured results):

| Tag | Size | `R_max` at 640x480 | `R_max` at 1280x720 |
| --- | --- | --- | --- |
| Outer | 22.7 cm | 7.5 m | 11 m |
| Middle | 4.5 cm | 1.5 m | 2.2 m |
| Inner | 1.6 cm | 0.5 m | 0.8 m |

These are model estimates, not measurements. They show the design rule: **each smaller tag must be detectable before the larger tag becomes unusable**. With the numbers above, the middle tag takes over from the outer tag when the drone is inside about 1.5-2 m, and the inner tag when it is under about 0.5-0.8 m, which matches the altitudes where the landing logic descends slowly and engages LAND.

## Choosing tag sizes for your own target

1. Pick the maximum altitude `R` at which you want the tag to be found.
2. Measure `p_min` for your camera, resolution and lighting with a test tag.
3. Compute `s = p_min * R / f_px`.
4. For each smaller tag, choose the distance `R_i` where it should take over, and compute its size with the same equation.
5. Check the larger tag still fits: the full tag must stay inside the image, which limits the minimum distance for a given size.
6. Print, measure the printed size, and flight-test.

## Tips for a longer range

- Use a larger tag, or a higher resolution (this also changes the image centre and PID scaling, see [software/config.md](software/config.md)).
- Keep the image sharp: fixed exposure and focus help, motion blur lowers detection range.
- Use matte print material and good contrast.

---

<div align="center">

| [⬅️ Previous: **04 - Nested AprilTags**](tags.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [➡️ Next: **06 - SITL Simulation**](simulation.md) |
| :--- | :---: | ---: |

</div>
