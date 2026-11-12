"""Render preview.png of the case (needs build123d, numpy, Pillow).

A small z-buffer rasteriser, so it runs anywhere without OpenGL.
python render_preview.py  -> preview.png next to this file
"""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from build123d import Align, Box, Pos

import display_case as dc

HERE = Path(__file__).parent


def tris(shape, tol=0.05):
    """Triangles (N x 3 x 3) of a build123d shape; faces that fail to mesh are skipped."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    w = shape.wrapped
    BRepMesh_IncrementalMesh(w, tol, False, 0.25, True)
    out = []
    ex = TopExp_Explorer(w, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None and tri.NbTriangles():
            tr = loc.Transformation()
            pts = np.array([tri.Node(i).Transformed(tr).Coord() for i in range(1, tri.NbNodes() + 1)])
            idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
            if f.Orientation() == TopAbs_REVERSED:
                idx = idx[:, ::-1]
            out.append(pts[idx])
        ex.Next()
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def display_model():
    """The vendor display model (jc1060p470c_display.step, case coordinates) or the simple envelope."""
    step = HERE / "jc1060p470c_display.step"
    if step.exists():
        from build123d import import_step

        return import_step(str(step)), "display model"
    return dc.build_display_dummy(), "display envelope"


def rot(elev, azim):
    """Camera rotation: azim about the case's Y (up) axis, then elev about X."""
    a, e = np.radians(azim), np.radians(elev)
    ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    return rx @ ry


def render(parts, elev, azim, size=(800, 560), pad=30):
    """parts: list of (triangles Nx3x3, rgb 0..1). Viewer looks down -Z of camera space."""
    R = rot(elev, azim)
    cam = [(t @ R.T, np.array(c)) for t, c in parts]
    allv = np.concatenate([t.reshape(-1, 3) for t, _ in cam])
    lo, hi = allv.min(0), allv.max(0)
    W, H = size
    s = min((W - 2 * pad) / (hi[0] - lo[0]), (H - 2 * pad) / (hi[1] - lo[1]))
    ox = W / 2 - s * (lo[0] + hi[0]) / 2
    oy = H / 2 + s * (lo[1] + hi[1]) / 2
    zbuf = np.full((H, W), -np.inf)
    img = np.ones((H, W, 3))
    light = np.array([-0.35, 0.55, 0.75])
    light /= np.linalg.norm(light)
    for t, col in cam:
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        k = 0.30 + 0.55 * np.abs(n @ light) + 0.15 * np.abs(n[:, 2])
        px = np.stack([ox + s * t[:, :, 0], oy - s * t[:, :, 1], t[:, :, 2]], axis=-1)
        for tri, kk in zip(px, k):
            x0, y0 = np.floor(tri[:, :2].min(0)).astype(int)
            x1, y1 = np.ceil(tri[:, :2].max(0)).astype(int)
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, W - 1), min(y1, H - 1)
            if x1 < x0 or y1 < y0:
                continue
            xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
            d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
            if abs(d) < 1e-9:
                continue
            w0 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / d
            w1 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / d
            w2 = 1 - w0 - w1
            m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            if not m.any():
                continue
            z = w0 * az + w1 * bz + w2 * cz
            zb = zbuf[y0 : y1 + 1, x0 : x1 + 1]
            upd = m & (z > zb)
            zb[upd] = z[upd]
            img[y0 : y1 + 1, x0 : x1 + 1][upd] = np.clip(col * kk, 0, 1)
    # Edge lines where depth jumps (silhouettes and steps).
    dz = np.zeros_like(zbuf)
    fin = np.where(np.isfinite(zbuf), zbuf, -1e3)
    dz[:-1] = np.maximum(dz[:-1], np.abs(np.diff(fin, axis=0)))
    dz[:, :-1] = np.maximum(dz[:, :-1], np.abs(np.diff(fin, axis=1)))
    img[dz > 1.5] *= 0.35
    return Image.fromarray((img * 255).astype(np.uint8))


def main():
    case = dc.build_case()
    disp, disp_name = display_model()
    grey, blue = (0.86, 0.86, 0.88), (0.35, 0.52, 0.85)
    case_t, disp_t = tris(case), tris(disp)
    disp_ex = tris(Pos(0, 0, -40) * disp)
    cut = Pos(0, -100 + 12.0, -60) * Box(300, 100, 200, align=(Align.CENTER, Align.MIN, Align.MIN))
    sec = [(tris(case & cut), grey), (tris(disp & cut), blue)]

    views = [
        ("Inside (front side): corner bosses, front lip", render([(case_t, grey)], elev=25, azim=200)),
        ("Back: USB-C window, screw counterbores, cable slot", render([(case_t, grey)], elev=25, azim=-25)),
        (f"Exploded: {disp_name} (blue) and case", render([(case_t, grey), (disp_ex, blue)], elev=22, azim=-35)),
        ("Section through the USB-C ports: 1 in. clear behind the bump-out", render(sec, elev=12, azim=-80)),
    ]
    W, H = views[0][1].size
    sheet = Image.new("RGB", (2 * W, 2 * (H + 34)), "white")
    dr = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
    for i, (title, im) in enumerate(views):
        x, y = (i % 2) * W, (i // 2) * (H + 34)
        dr.text((x + 20, y + 8), title, fill="black", font=font)
        sheet.paste(im, (x, y + 34))
    out = HERE / "preview.png"
    sheet.save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
