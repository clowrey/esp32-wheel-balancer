# Display rear case (build123d)

3D-printable rear case for the Guition JC1060P470C_I_W_Y 7" display. It screws
onto the display's four corner standoffs, its outline matches the glass
(181.80 x 113.72 mm, same corner radius), and it leaves 1 in. (25.4 mm) of
free depth behind the raised back cover for the sensor and tach wiring.

![preview](preview.png)

| File | What |
|---|---|
| `display_case.py` | The model. All dimensions are dataclass fields at the top. |
| `display_case.step` | Default build: flush outline, 1 in. clearance, front lip, side tabs. |
| `render_preview.py` | Writes `preview.png` (numpy z-buffer renderer, no OpenGL). |
| `JC1060P470C_I_W_Y-structure-drawing.pdf` | Vendor structure drawing the dimensions come from. |

```
pip install build123d
python display_case.py                      # flush with the glass (default)
python display_case.py --overhang 6.35      # 1/4 in. bigger all round; lip then wraps the glass edge
python display_case.py --clearance 30       # more room behind the bump-out
python display_case.py --no-lip --no-tabs
python display_case.py --stl                # also write an STL
python render_preview.py
```

## What the case has

- Overall 211.80 x 113.72 x 40.4 mm with the default settings (181.80 wide
  without the tabs): 36.2 mm behind the standoff plane plus a 4.2 mm lip
  forward of it. Display plus case is 48.2 mm deep.
- Four corner bosses at 166.00 x 99.00 mm, tied into the corner walls. A 2.4 mm
  M2 through hole with a 4.6 mm counterbore from the back, leaving a 4 mm
  floor. An M2 x 8 screw from the back then has about 4 mm in the standoff. A
  long screwdriver reaches down the counterbore.
- Front lip, 1.6 mm thick, that carries the wall forward from the standoff
  tops to 0.8 mm short of the back face of the plastic bezel, so the edge gap
  is closed. With `--overhang` of 2 mm or more the lip instead wraps the bezel
  and ends flush with the glass front.
- Mounting tabs on the left and right: 5 mm thick, 15 mm out from the wall,
  40 mm long, flush with the back face, each with a 4.5 mm hole (M4 / #8).
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
| Standoff OD | 5.5 mm | scaled | fine (case boss is 9 mm) |
| Standoff thread | M2 | Chester, on the unit | good |
| Glass front to back face of bezel | 7 mm | Chester, on the unit | good |
| Standoffs past the bezel | 5 mm (so 12 mm from the glass front) | Chester, on the unit; the drawing says 8.00 total | good |
| Back cover (bump-out) | 124.66 x 74.67 mm, R5.2, centred | printed; radius scaled | good |
| Bump-out height above standoff tops | 8.38 mm | printed, but the drawing's other depth was off by 4 mm | **check** |
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

1. How deep the standoff threads go. M2 x 8 puts about 4 mm in the standoff;
   use M2 x 6 if the hole is shallower.
2. Bump-out height above the standoff tops (8.38 mm here). The 1 in.
   clearance is measured from there; if it is really 4.4 mm (16.38 from the
   glass front, as the drawing implies), the case just has 4 mm extra room.
3. USB-C position: hold a ruler over the back, compare with the 16.8 x 35.1 mm
   window at 47.8 mm right of centre.

## 3D model of the display

No STEP or STL of this display turned up online (searched GitHub, MakerWorld,
Printables, Thingiverse, GrabCAD aggregators). The vendor pack's
`JC1060P470C_I_W_Y.dwg` (14 MB, AutoCAD 2018 format, same mirror as above)
contains `3DSolid` entities, so it probably holds a 3D model of the board.
Opening it in Fusion 360, AutoCAD or DraftSight and exporting STEP should
work; it could not be converted here because no open-source tool reads ACIS
solids. `build_display_dummy()` in the script is a simple envelope built from
the drawing and the measurements above.
