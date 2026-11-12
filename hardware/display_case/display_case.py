"""Rear case for the Guition JC1060P470C_I_W_Y (ESP32-P4, 7" 1024x600).

The case screws onto the display's four threaded corner standoffs and gives
`clearance_behind_bump` of free space behind the board's raised back cover
(the "bump-out") for wiring. See README.md for where every board dimension
came from and which ones to check with calipers before printing.

Coordinates (all mm):
  * X/Y are as seen from the BACK of the display (the side the case is on):
    +X right, +Y up. Origin = centre of the glass outline. From the back,
    the USB-C pair is on the right and the JP1 header on the left, as in the
    spec-sheet back view (the drawing's "bottom view" is mirrored).
    Feature heights (Y) assume the drawing's top edge is the camera edge;
    the case is symmetric enough that only the USB-C window and cable slot
    move if that is wrong (see README).
  * Z = 0 is the top face of the four standoffs (the plane the case sits on).
    +Z points out of the back of the display, towards the case's back wall.

Print it back-wall down on the bed: walls and the front lip grow upward, the
screw counterbores open onto the bed, and no supports are needed.

Run:  python display_case.py            -> writes display_case.step next to this file (--stl for an STL too)
      python display_case.py --no-lip   -> case without the front lip
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Align,
    Box,
    Cylinder,
    Part,
    Pos,
    Rectangle,
    RectangleRounded,
    Rot,
    SlotOverall,
    export_step,
    export_stl,
    extrude,
)

IN = 25.4


# --------------------------------------------------------------------------
# Board dimensions. Source: vendor structure drawing
# "JC1060P470C_I_W_Y-模型.pdf" (Guition, AutoCAD, 2025-09-08), measured from
# its vector geometry, plus the numbers printed on it. README.md has the list.
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Board:
    glass_w: float = 181.80          # printed on drawing + spec sheet
    glass_h: float = 113.72          # printed on drawing + spec sheet
    glass_r: float = 4.57            # corner arc in drawing (not printed)
    standoff_dx: float = 166.00      # printed, hole spacing X
    standoff_dy: float = 99.00       # printed, hole spacing Y
    standoff_od: float = 5.53        # drawn standoff boss (not printed)
    # Depths from the vendor's 3D model (converted from the DWG, see
    # README): bezel back face 8.00 mm behind the glass front, standoffs
    # 5.00 mm proud of that. Chester measured 7 + 5 on the unit; the lip only
    # depends on the 5 mm, which both agree on.
    front_to_standoff: float = 13.0  # glass front -> standoff tops
    bezel_thk: float = 8.0           # glass front -> back face of the bezel
    bump_w: float = 124.66           # printed (back cover width)
    bump_h: float = 74.67            # printed (back cover height)
    bump_r: float = 5.21             # drawn corner (not printed)
    bump_dz: float = 3.37            # 3D model: standoff tops -> back of the cover (16.37 from the glass front)
    bump_cx: float = -0.15           # cover centre offset, drawing (tiny)
    bump_cy: float = 0.0
    rj45_dz: float = 8.31            # 3D model: RJ45 sticks out 8.3 mm past the cover
    # USB-C pair (two vertical receptacles in oval cut-outs of the cover),
    # back-view coordinates of the pair's bounding box.
    usb_x0: float = 45.5             # 3D model
    usb_x1: float = 50.2
    usb_y0: float = 1.6
    usb_y1: float = 23.3
    # 26-pin JP1 header (2x13), back-view coordinates.
    hdr_x0: float = -58.5
    hdr_x1: float = -52.4
    hdr_y0: float = -17.3
    hdr_y1: float = 16.8


# --------------------------------------------------------------------------
# Case parameters (edit these)
# --------------------------------------------------------------------------
@dataclass
class CaseParams:
    overhang: float = 0.0              # outline grows this much past the glass (0 = flush, 1/4" = 6.35)
    clearance_behind_bump: float = 1.0 * IN  # free depth behind the bump-out
    wall: float = 2.4                  # side wall thickness
    back: float = 2.4                  # back wall thickness
    # Front lip: extends the side wall from the standoff plane towards the
    # bezel so the gap round the edge is closed. Flush case: thin lip that
    # stops short of the bezel's back face. Overhanging case: lip wraps the
    # bezel edge and ends flush with the glass front.
    lip: bool = True
    lip_thk: float = 1.6
    lip_gap_to_bezel: float = 0.8      # flush case: air gap left before the bezel's back face
    wrap_clearance: float = 0.4        # overhang case: gap round the glass edge
    # Mounting screws, from the back into the display standoffs (M2, confirmed).
    hole_d: float = 2.4                # M2 clearance
    cbore_d: float = 4.6               # M2 socket or pan head (3.8-4.0 mm) + clearance
    screw_floor: float = 4.0           # material under the head: M2x8 -> ~4 mm in the standoff
    boss_d: float = 9.0
    # USB-C access window in the back wall, over the two ports.
    usb_window: bool = True
    usb_margin: float = 5.0            # window grows this much round the port pair
    # Cable exit for the sensor / tach harness, in the side wall next to JP1.
    cable_slot: bool = True
    cable_slot_len: float = 22.0       # along Y
    cable_slot_w: float = 10.0         # along Z
    cable_slot_side: str = "-x"        # "-x" (JP1 side, left from the back) or "+x"
    # Mounting tabs on the left and right walls, flush with the back face.
    tabs: bool = True
    tab_thk: float = 5.0
    tab_out: float = 15.0              # how far each tab sticks out past the wall
    tab_len_frac: float = 0.8          # tab length along Y as a fraction of the case height
    tab_r: float = 4.0                 # outer corner radius
    tab_hole_d: float = 4.5            # M4 / #8 clearance; 0 for no holes
    tab_holes: int = 2                 # holes per tab
    tab_hole_edge: float = 10.0        # end holes' centre distance from the tab ends


def build_case(p: CaseParams = CaseParams(), b: Board = Board()) -> Part:
    out_w = b.glass_w + 2 * p.overhang
    out_h = b.glass_h + 2 * p.overhang
    out_r = b.glass_r + p.overhang

    inner_depth = b.bump_dz + p.clearance_behind_bump   # standoff plane -> inside of back wall
    total_z = inner_depth + p.back

    # ---- shell: closed box, then hollow it out --------------------------
    outer = extrude(RectangleRounded(out_w, out_h, out_r), amount=total_z)
    in_w, in_h = out_w - 2 * p.wall, out_h - 2 * p.wall
    in_r = max(out_r - p.wall, 0.5)
    cavity = extrude(RectangleRounded(in_w, in_h, in_r), amount=inner_depth)
    case = outer - cavity

    # ---- front lip -------------------------------------------------------
    if p.lip:
        wrap = p.overhang >= p.lip_thk + p.wrap_clearance
        if wrap:
            # Overhanging case: lip goes round the glass edge, flush with the front.
            lip_depth = b.front_to_standoff
            lip_in_w = b.glass_w + 2 * p.wrap_clearance
            lip_in_h = b.glass_h + 2 * p.wrap_clearance
            lip_in_r = b.glass_r + p.wrap_clearance
        else:
            # Flush case: thin lip behind the bezel, stops short of it.
            lip_depth = b.front_to_standoff - b.bezel_thk - p.lip_gap_to_bezel
            lip_in_w, lip_in_h = out_w - 2 * p.lip_thk, out_h - 2 * p.lip_thk
            lip_in_r = max(out_r - p.lip_thk, 0.5)
        if lip_depth > 0.2:
            lip = extrude(RectangleRounded(out_w, out_h, out_r), amount=lip_depth) - extrude(
                RectangleRounded(lip_in_w, lip_in_h, lip_in_r), amount=lip_depth
            )
            case += Pos(0, 0, -lip_depth) * lip

    # ---- corner bosses + screw holes --------------------------------------
    sx, sy = b.standoff_dx / 2, b.standoff_dy / 2
    for ix in (-1, 1):
        for iy in (-1, 1):
            cx, cy = ix * sx, iy * sy
            boss = Pos(cx, cy, 0) * Cylinder(
                p.boss_d / 2, inner_depth, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )
            # Web tying the boss into the corner walls (stiffer, prints cleaner).
            web_x = (out_w / 2 - p.wall / 2) - cx * ix
            web_y = (out_h / 2 - p.wall / 2) - cy * iy
            web = Pos(cx + ix * web_x / 2, cy + iy * web_y / 2, 0) * Box(
                web_x, web_y, inner_depth, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )
            # Keep the web inside the outline.
            web = web & extrude(RectangleRounded(out_w, out_h, out_r), amount=inner_depth)
            case += boss + web
            # Through hole for the screw shank.
            case -= Pos(cx, cy, -1) * Cylinder(
                p.hole_d / 2, total_z + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )
            # Counterbore from the back so a short screw reaches the standoff.
            case -= Pos(cx, cy, p.screw_floor) * Cylinder(
                p.cbore_d / 2, total_z, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )

    # ---- USB-C window in the back wall ------------------------------------
    if p.usb_window:
        ww = (b.usb_x1 - b.usb_x0) + 2 * p.usb_margin
        wh = (b.usb_y1 - b.usb_y0) + 2 * p.usb_margin
        wx, wy = (b.usb_x0 + b.usb_x1) / 2, (b.usb_y0 + b.usb_y1) / 2
        case -= Pos(wx, wy, inner_depth - 1) * extrude(RectangleRounded(ww, wh, 3.0), amount=p.back + 2)

    # ---- cable slot in the side wall next to JP1 ---------------------------
    if p.cable_slot:
        side = 1 if p.cable_slot_side == "+x" else -1
        slot_z = inner_depth - p.cable_slot_w / 2 - 3.0    # near the back, above the bump
        slot_y = (b.hdr_y0 + b.hdr_y1) / 2
        slot = extrude(SlotOverall(p.cable_slot_len, p.cable_slot_w), amount=p.wall * 4, both=True)
        # Slot sketch is in XY with length along X; turn it to lie in the YZ wall.
        slot = Rot(0, 90, 0) * Rot(0, 0, 90) * slot
        case -= Pos(side * (out_w / 2), slot_y, slot_z) * slot

    # ---- mounting tabs, left and right, flush with the back face ----------
    if p.tabs:
        for side in (-1, 1):
            # Overlap the wall by its thickness so the tab fuses into it.
            tab_len = p.tab_len_frac * out_h
            tw = p.tab_out + p.wall
            tx = side * (out_w / 2 + p.tab_out / 2 - p.wall / 2)
            tab = Pos(tx, 0, total_z - p.tab_thk) * extrude(RectangleRounded(tw, tab_len, p.tab_r), amount=p.tab_thk)
            # Square off the inner corners where the tab meets the wall.
            tab += Pos(side * (out_w / 2 - p.wall / 2 + p.tab_r / 2), 0, total_z - p.tab_thk) * extrude(
                Rectangle(p.wall + p.tab_r, tab_len), amount=p.tab_thk
            )
            case += tab
            if p.tab_hole_d > 0 and p.tab_holes > 0:
                hx = side * (out_w / 2 + p.tab_out / 2)
                span = tab_len / 2 - p.tab_hole_edge
                ys = [0.0] if p.tab_holes == 1 else [
                    -span + 2 * span * i / (p.tab_holes - 1) for i in range(p.tab_holes)
                ]
                for hy in ys:
                    case -= Pos(hx, hy, total_z - p.tab_thk - 1) * Cylinder(
                        p.tab_hole_d / 2, p.tab_thk + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)
                    )

    return case


def build_display_dummy(b: Board = Board()) -> Part:
    """Rough display envelope for previews and fit checks (not for printing)."""
    glass = Pos(0, 0, -b.front_to_standoff) * extrude(
        RectangleRounded(b.glass_w, b.glass_h, b.glass_r), amount=b.bezel_thk
    )
    body = Pos(0, 0, -b.front_to_standoff + b.bezel_thk) * extrude(
        RectangleRounded(165.0, 100.3, 2.0), amount=b.front_to_standoff - b.bezel_thk
    )
    bump = Pos(b.bump_cx, b.bump_cy, 0) * extrude(RectangleRounded(b.bump_w, b.bump_h, b.bump_r), amount=b.bump_dz)
    rj45 = Pos(30.25, 4.43, b.bump_dz) * extrude(RectangleRounded(16.71, 16.12, 0.5), amount=b.rj45_dz)
    hdr = Pos((b.hdr_x0 + b.hdr_x1) / 2, (b.hdr_y0 + b.hdr_y1) / 2, b.bump_dz) * extrude(
        RectangleRounded(b.hdr_x1 - b.hdr_x0, b.hdr_y1 - b.hdr_y0, 0.5), amount=8.5
    )
    standoffs = Part()
    for ix in (-1, 1):
        for iy in (-1, 1):
            standoffs += Pos(ix * b.standoff_dx / 2, iy * b.standoff_dy / 2, -b.front_to_standoff + b.bezel_thk) * Cylinder(
                b.standoff_od / 2, b.front_to_standoff - b.bezel_thk, align=(Align.CENTER, Align.CENTER, Align.MIN)
            )
    return glass + body + bump + rj45 + hdr + standoffs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--overhang", type=float, default=0.0, help="mm past the glass edge (0 = flush)")
    ap.add_argument("--clearance", type=float, default=IN, help="mm of free depth behind the bump-out")
    ap.add_argument("--no-lip", action="store_true")
    ap.add_argument("--no-tabs", action="store_true")
    ap.add_argument("--stl", action="store_true", help="also write an STL")
    ap.add_argument("--out", default=str(Path(__file__).with_name("display_case")), help="output path stem")
    a = ap.parse_args()

    p = CaseParams(overhang=a.overhang, clearance_behind_bump=a.clearance, lip=not a.no_lip, tabs=not a.no_tabs)
    case = build_case(p)
    export_step(case, a.out + ".step")
    if a.stl:
        export_stl(case, a.out + ".stl", tolerance=0.02, angular_tolerance=0.2)
    bb = case.bounding_box()
    print(f"case: {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, volume {case.volume / 1000:.1f} cm^3")
    print(f"wrote {a.out}.step" + (f" and {a.out}.stl" if a.stl else ""))


if __name__ == "__main__":
    main()
