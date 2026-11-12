# Display rear case (build123d)

3D-printable rear case for the Guition JC1060P470C_I_W_Y 7" display. It screws
onto the display's four corner standoffs, its outline matches the glass
(181.80 x 113.72 mm, same corner radius), and it leaves 1 in. (25.4 mm) of
free depth behind the raised back cover for the sensor and tach wiring.

![preview](preview.png)

| File | What |
|---|---|
| `display_case.py` | The model. All dimensions are dataclass fields at the top. |
| `display_case.step` / `.stl` | Default build: flush outline, 1 in. clearance, front lip. |
| `render_preview.py` | Writes `preview.png` (numpy z-buffer renderer, no OpenGL). |
| `JC1060P470C_I_W_Y-structure-drawing.pdf` | Vendor structure drawing the dimensions come from. |

```
pip install build123d
python display_case.py                      # flush with the glass (default)
python display_case.py --overhang 6.35      # 1/4 in. bigger all round; lip then wraps the glass edge
python display_case.py --clearance 30       # more room behind the bump-out
python display_case.py --no-lip
python render_preview.py
```

## What the case has

- Overall 181.80 x 113.72 x 42.1 mm with the default settings: 36.2 mm
  behind the standoff plane plus a 5.9 mm lip forward of it.
- Four corner bosses at 166.00 x 99.00 mm, tied into the corner walls. A 2.9 mm
  through hole (clears M2 or M2.5) with a 5.6 mm counterbore from the back,
  leaving a 4 mm floor. An M2 x 8 (or M2.5 x 8) screw from the back then has
  about 4 mm in the standoff. A long screwdriver reaches down the counterbore.
- Front lip, 1.6 mm thick, that carries the wall forward from the standoff
  plane to 0.8 mm short of the back of the glass, so the edge gap is closed.
  With `--overhang` of 2 mm or more the lip instead wraps the glass edge and
  ends flush with the front.
- USB-C window (16.8 x 35.1 mm) in the back wall over the two USB-C ports, so
  a cable plugs straight in for flashing. The ports point out of the back, and
  a normal plug's overmold ends inside the case, so the cable leaves through
  the window.
- 22 x 10 mm cable slot in the side wall next to the JP1 header (the left side
  when you look at the back) for the sensor and tach harness.
- Print back-wall down. No supports. 2.4 mm walls; PETG or PLA is fine.

Coordinates in the script are as seen from the back of the display: +X right,
+Y up, Z = 0 at the standoff tops, +Z out the back.

## Where each dimension came from

The source is Guition's structure drawing `JC1060P470C_I_W_Y-模型.pdf`
(AutoCAD, dated 2025-09-08). It ships in the vendor's download pack; the copy
used here came from the public mirror `wegi1/ESP32P4-JC1060P470C-I_W_Y`
(`3-Structure_Diagram/`, next to the same V1.0 schematic this repo has). The
pack also has a `.dwg` of the same drawing. Numbers marked *printed* are
dimension labels on the drawing. Numbers marked *scaled* were measured from the
drawing's vector geometry, scaled by the printed 181.80 mm width (the printed
heights 113.72, 99.00 and 74.67 all check out within 0.03 mm at that scale).

| Dimension | Value | Source | Confidence |
|---|---|---|---|
| Glass outline | 181.80 x 113.72 mm | printed; also on the spec sheet | good |
| Glass corner radius | 4.57 mm | scaled | check |
| Standoff spacing | 166.00 x 99.00 mm, centred | printed | good |
| Standoff hole | Ø2.00 (4x) | printed; the drawn inner circle is 2.5 mm | **check M2 vs M2.5** |
| Standoff OD | 5.5 mm | scaled | fine (case boss is 9 mm) |
| Front of glass to standoff tops | 8.00 mm | printed (side view) | check |
| Glass thickness at the edge | 1.32 mm | scaled (side view) | **check** (sets the lip) |
| Back cover (bump-out) | 124.66 x 74.67 mm, R5.2, centred | printed; radius scaled | good |
| Bump-out height above standoff tops | 8.38 mm | printed | check |
| RJ45 sticks out past the cover | 4.00 mm | printed (17 x 16 mm, scaled) | clears easily |
| USB-C pair | 6.8 x 25.1 mm box, 47.8 mm right of centre and 12.5 mm above it (from the back) | scaled | check |
| JP1 header | 55.5 mm left of centre, vertically centred (from the back) | scaled | not critical |

Left/right: the drawing's "bottom view" is mirrored compared with the real
back of the board. The spec sheet's back render and the bare-board photo both
put the USB-C ports on the right and JP1 on the left when you look at the
back, so the case follows those. Up/down: the case assumes the drawing's top
edge is the camera edge of the screen. Only the USB-C window and the cable
slot depend on this; if the window lands on the wrong half, set
`usb_y0 = -25.0` and `usb_y1 = 0.1` in `Board` and re-run.

## Measure before printing

1. Standoff thread: M2 or M2.5? The 2.9 mm hole clears both; just buy the
   right screws.
2. Glass front to standoff tops (8.00 mm) and the glass thickness at the edge
   (1.32 mm). The lip depth is 8.00 - 1.32 - 0.8 = 5.9 mm. If anything sits
   behind the glass edge inside the outer 1.6 mm band, set `lip=False` or
   shorten the lip.
3. Bump-out height above the standoff tops (8.38 mm). The 1 in. clearance is
   measured from there.
4. USB-C position: hold a ruler over the back, compare with the 16.8 x 35.1 mm
   window at 47.8 mm right of centre.
