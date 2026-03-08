#!/usr/bin/env python3
"""
Generate an excavation pit STL from MathWorks autonomous-excavator Membrane_points.csv.
Reads the 2D contour, scales it to meter size, and extrudes it downward to form a pit.
Usage: run from repo root or pass path to Membrane_points.csv; writes excavation_pit.stl to cwd or --output.
"""
import csv
import argparse
import os

def load_contour(path):
    pts = []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            x = float(row['x'])
            y = float(row['y'])
            pts.append((x, y))
    return pts

def write_stl(triangles, outpath):
    """triangles: list of ((x,y,z), (x,y,z), (x,y,z))"""
    with open(outpath, 'w') as f:
        f.write("solid excavation_pit\n")
        for (a, b, c) in triangles:
            # normal from cross product (b-a) x (c-a)
            v1 = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
            v2 = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
            nx = v1[1]*v2[2] - v1[2]*v2[1]
            ny = v1[2]*v2[0] - v1[0]*v2[2]
            nz = v1[0]*v2[1] - v1[1]*v2[0]
            L = (nx*nx+ny*ny+nz*nz)**0.5
            if L > 1e-10:
                nx, ny, nz = nx/L, ny/L, nz/L
            else:
                nx, ny, nz = 0, 0, 1
            f.write("  facet normal {} {} {}\n".format(nx, ny, nz))
            f.write("    outer loop\n")
            f.write("      vertex {} {} {}\n".format(a[0], a[1], a[2]))
            f.write("      vertex {} {} {}\n".format(b[0], b[1], b[2]))
            f.write("      vertex {} {} {}\n".format(c[0], c[1], c[2]))
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write("endsolid excavation_pit\n")

def main():
    ap = argparse.ArgumentParser(description="Convert Membrane_points.csv to pit STL")
    ap.add_argument("csv", nargs="?", default=None, help="Path to Membrane_points.csv")
    ap.add_argument("-o", "--output", default="excavation_pit.stl", help="Output STL path")
    ap.add_argument("--scale", type=float, default=150.0, help="Scale contour to meters (default 150)")
    ap.add_argument("--depth", type=float, default=2.0, help="Pit depth in meters (default 2.0)")
    args = ap.parse_args()

    if args.csv is None:
        # Try repo path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg = os.path.dirname(script_dir)
        default_csv = os.path.join(pkg, "..", "mathworks_autonomous_excavator", "ExcavatorCADFiles", "Membrane_points.csv")
        args.csv = os.path.normpath(os.path.join(pkg, default_csv))
    if not os.path.isfile(args.csv):
        print("CSV not found:", args.csv)
        return 1

    pts = load_contour(args.csv)
    if len(pts) < 3:
        print("Need at least 3 points")
        return 1

    s = args.scale
    d = args.depth
    top = [(p[0]*s, p[1]*s, 0.0) for p in pts]
    bot = [(p[0]*s, p[1]*s, -d) for p in pts]
    n = len(pts)
    cx = sum(p[0] for p in top) / n
    cy = sum(p[1] for p in top) / n
    c = (cx, cy, 0.0)
    cb = (cx, cy, -d)

    triangles = []
    for i in range(n):
        j = (i + 1) % n
        # top cap (triangle from centroid)
        triangles.append((top[i], top[j], c))
        # bottom cap
        triangles.append((bot[j], bot[i], cb))
        # side quad -> two triangles
        triangles.append((top[i], bot[i], top[j]))
        triangles.append((top[j], bot[i], bot[j]))

    write_stl(triangles, args.output)
    print("Wrote", args.output, "({} triangles)".format(len(triangles)))
    return 0

if __name__ == "__main__":
    exit(main())
