# Nested Tags

The landing target is **one printed target made of three AprilTags of different sizes**, one inside the other. This keeps a detectable tag in view from cruise altitude down to the ground.

## Why nested

A single tag is either big enough to see from altitude but leaves the camera's view or becomes undetectable when the drone is close, or small enough for close range but invisible from far away. Three sizes cover the whole descent.

## Current design

| Tag | Family | ID | Size | Used for |
| --- | --- | --- | --- | --- |
| Outer | 48h12 | 0 | 22.7 cm | Long range: detected up to about 7-8 m at 640x480, 10-12 m at 1280x720 |
| Middle | 48h12 | 1 | 4.5 cm | Medium and close range |
| Inner | 48h12 | 2 | 1.6 cm | Final close range |

TODO: add the tag image here (for example `images/tag_0_1_2.png`). The tags are placed concentrically: tag 1 inside tag 0, tag 2 inside tag 1.

Confirm the sizes you print by measuring the printed tag edge with a ruler. Detection range scales with printed size, see [detection-range.md](detection-range.md).

The tag IDs and sizes change between experiments, so nothing in the landing code is tied to a specific ID.

## Printing

- Print on matte, non-reflective material. Glossy paper and laminate create glare that hides the tag.
- Keep the print flat and rigid (mount it on a board).
- Keep a clean white margin around the outer tag.
- Do not scale the print to fit a page: measure the result.

## Generating tags

Use the AprilTag images from the AprilRobotics project for the 48h12 family and scale them to the sizes above. TODO: add the exact generation steps or a script you used.

## Changing tag IDs or family

1. In [../config/params.yaml](../config/params.yaml), set `tag_ids` (for example `[6, 7, 8]`) and `tag_priorities` in the same order.
2. In the `apriltag_ros` configuration (Docker repository) set the same family and IDs, and the tag sizes if you use pose output.
3. Rebuild nothing: the landing server only uses tags in the goal list and ignores all other tags.

## Priorities

When more than one listed tag is visible, the tag with the highest priority value is tracked. With the defaults:

```yaml
tag_ids:        [0,   1,   2  ]
tag_priorities: [1.0, 0.8, 0.6]
```

the outer tag (0) wins whenever it is visible, and the middle and inner tags take over only when the outer tag is not detected. If you would rather track the smallest visible tag as soon as it appears, reverse the priorities, for example `[0.6, 0.8, 1.0]`. The comment in `mission_executive.py` describes the second behaviour, see [known-issues.md](known-issues.md).
