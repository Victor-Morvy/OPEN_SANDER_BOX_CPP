"""
AC3D (.ac) to OBJ converter with animated-part extraction.
Usage: python ac2obj.py input.ac output.obj

Outputs:
  output.obj          — static mesh (animated parts excluded)
  output.mtl          — shared material file
  erj195_PART.obj     — one file per animated part (same dir)
  erj195_parts.json   — pivot / axis descriptor for each part
"""

import sys, os, math, json

# ── Animated part names ───────────────────────────────────────────────────────

ANIM_NAMES = {
    "aileron.l", "aileron.r",
    "elevator.l", "elevator.r",
    "rudder",
    "flap.l.1", "flap.l.2", "flap.r.1", "flap.r.2",
    "spoiler.l.1", "spoiler.l.2", "spoiler.r.1", "spoiler.r.2",
    "gear.l", "gear.r", "gear.n",
    "tire.l", "tire.r", "tire.n",
    "geardoor.b.l", "geardoor.b.r", "geardoor.f.l", "geardoor.f.r",
    "fan.l", "fan.r",
    "reverser",
}

# ── AC3D parser ───────────────────────────────────────────────────────────────

def parse_ac(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    if not lines or not lines[0].startswith("AC3D"):
        raise ValueError("Not an AC3D file")

    materials = []
    root_objects = []
    i = 1

    def read_material(line):
        parts = line.split()
        mat = {"name": parts[1].strip('"')}
        try:
            ri = parts.index("rgb");  mat["rgb"]  = (float(parts[ri+1]), float(parts[ri+2]), float(parts[ri+3]))
            ai = parts.index("amb");  mat["amb"]  = (float(parts[ai+1]), float(parts[ai+2]), float(parts[ai+3]))
            ti = parts.index("trans"); mat["trans"] = float(parts[ti+1])
        except (ValueError, IndexError):
            mat["rgb"] = (1,1,1); mat["amb"] = (1,1,1); mat["trans"] = 0.0
        return mat

    def read_object():
        nonlocal i
        obj = {
            "type": None, "name": "", "loc": (0,0,0), "rot": None,
            "texture": None, "texrep": (1,1), "crease": 45,
            "verts": [], "surfs": [], "kids": []
        }
        while i < len(lines):
            tok = lines[i].rstrip(); i += 1
            if not tok.strip(): continue
            key = tok.split()[0] if tok.split() else ""
            if key == "name":
                obj["name"] = tok.split(None, 1)[1].strip().strip('"')
            elif key == "loc":
                p = tok.split(); obj["loc"] = (float(p[1]), float(p[2]), float(p[3]))
            elif key == "rot":
                p = tok.split(); obj["rot"] = [float(x) for x in p[1:10]]
            elif key == "texture":
                obj["texture"] = tok.split(None, 1)[1].strip().strip('"')
            elif key == "texrep":
                p = tok.split(); obj["texrep"] = (float(p[1]), float(p[2]))
            elif key == "crease":
                obj["crease"] = float(tok.split()[1])
            elif key == "numvert":
                n = int(tok.split()[1])
                for _ in range(n):
                    p = lines[i].split(); i += 1
                    obj["verts"].append((float(p[0]), float(p[1]), float(p[2])))
            elif key == "numsurf":
                n = int(tok.split()[1])
                for _ in range(n):
                    surf = {"flags": 0, "mat": 0, "refs": []}
                    while i < len(lines):
                        sl = lines[i].rstrip(); i += 1
                        if not sl.strip(): continue
                        sk = sl.split()[0]
                        if sk == "SURF":   surf["flags"] = int(sl.split()[1], 16)
                        elif sk == "mat":  surf["mat"]   = int(sl.split()[1])
                        elif sk == "refs":
                            nr = int(sl.split()[1])
                            for _ in range(nr):
                                rp = lines[i].split(); i += 1
                                surf["refs"].append((int(rp[0]), float(rp[1]), float(rp[2])))
                            break
                    obj["surfs"].append(surf)
            elif key == "kids":
                n = int(tok.split()[1])
                for _ in range(n):
                    while i < len(lines) and not lines[i].strip().startswith("OBJECT"):
                        i += 1
                    if i >= len(lines): break
                    i += 1
                    child = read_object()
                    obj["kids"].append(child)
                break
        return obj

    while i < len(lines):
        line = lines[i].rstrip(); i += 1
        if not line.strip(): continue
        if line.startswith("MATERIAL"):
            materials.append(read_material(line))
        elif line.startswith("OBJECT"):
            root_objects.append(read_object())

    return materials, root_objects


# ── Transform helpers ─────────────────────────────────────────────────────────

def compose(parent_xf, loc, rot):
    if rot is None:
        rot = [1,0,0, 0,1,0, 0,0,1]
    ox, oy, oz = loc
    pr = parent_xf[1]
    wx = pr[0]*ox + pr[1]*oy + pr[2]*oz + parent_xf[0][0]
    wy = pr[3]*ox + pr[4]*oy + pr[5]*oz + parent_xf[0][1]
    wz = pr[6]*ox + pr[7]*oy + pr[8]*oz + parent_xf[0][2]
    def matmul(a, b):
        r = [0]*9
        for row in range(3):
            for col in range(3):
                for k in range(3):
                    r[row*3+col] += a[row*3+k] * b[k*3+col]
        return r
    return ((wx, wy, wz), matmul(pr, rot))

def transform_vertex(xf, v):
    ox, oy, oz = xf[0]; r = xf[1]; x, y, z = v
    return (r[0]*x + r[1]*y + r[2]*z + ox,
            r[3]*x + r[4]*y + r[5]*z + oy,
            r[6]*x + r[7]*y + r[8]*z + oz)

def ac_to_gl(ax, ay, az):
    """AC3D world → OpenGL aircraft body: GL.x=-AC.z, GL.y=AC.y, GL.z=AC.x"""
    return (-az, ay, ax)


# ── OBJ writer ────────────────────────────────────────────────────────────────

class ObjWriter:
    def __init__(self):
        self.verts   = []
        self.uvs     = []
        self.normals = []
        self.groups  = {}

    def add_group(self, mat_idx):
        if mat_idx not in self.groups:
            self.groups[mat_idx] = []

    def flat_normal(self, pts):
        if len(pts) < 3: return (0, 1, 0)
        ax, ay, az = pts[1][0]-pts[0][0], pts[1][1]-pts[0][1], pts[1][2]-pts[0][2]
        bx, by, bz = pts[2][0]-pts[0][0], pts[2][1]-pts[0][1], pts[2][2]-pts[0][2]
        nx = ay*bz - az*by; ny = az*bx - ax*bz; nz = ax*by - ay*bx
        l = math.sqrt(nx*nx + ny*ny + nz*nz)
        return (nx/l, ny/l, nz/l) if l > 1e-10 else (0, 1, 0)

    def add_poly(self, mat_idx, world_pts, uvs, texrep):
        if len(world_pts) < 3: return
        self.add_group(mat_idx)
        ni = len(self.normals)
        self.normals.append(self.flat_normal(world_pts))
        vi_list, vti_list = [], []
        for (wx, wy, wz), (tu, tv) in zip(world_pts, uvs):
            vi = len(self.verts);  vti = len(self.uvs)
            self.verts.append((wx, wy, wz))
            self.uvs.append((tu * texrep[0], (1.0 - tv) * texrep[1]))
            vi_list.append(vi); vti_list.append(vti)
        for t in range(1, len(vi_list) - 1):
            self.groups[mat_idx].append([
                (vi_list[0],   vti_list[0],   ni),
                (vi_list[t],   vti_list[t],   ni),
                (vi_list[t+1], vti_list[t+1], ni),
            ])

    def tri_count(self):
        return sum(len(g) for g in self.groups.values())

    def gl_verts(self):
        """Return vertices remapped to GL space."""
        return [ac_to_gl(x, y, z) for x, y, z in self.verts]


# ── Geometry flattener ────────────────────────────────────────────────────────

def _add_surfs(obj, xf, writer):
    """Add an object's surfaces (world-space) to writer. No recursion."""
    texrep = obj.get("texrep", (1, 1))
    if not (obj["verts"] and obj["surfs"]): return
    world_verts = [transform_vertex(xf, v) for v in obj["verts"]]
    for surf in obj["surfs"]:
        flags = surf["flags"]
        refs  = surf["refs"]
        if len(refs) < 2: continue
        stype = flags & 0x0F
        if stype not in (0, 0x10): continue  # skip line strips
        pts = [world_verts[r[0]] for r in refs]
        uvs = [(r[1], r[2]) for r in refs]
        writer.add_poly(surf["mat"], pts, uvs, texrep)
        if flags & 0x20:  # two-sided
            writer.add_poly(surf["mat"], list(reversed(pts)), list(reversed(uvs)), texrep)

def _flatten_subtree(obj, parent_xf, writer):
    """Flatten obj and all children into writer. Used for animated parts."""
    xf = compose(parent_xf, obj["loc"], obj["rot"])
    _add_surfs(obj, xf, writer)
    for kid in obj["kids"]:
        _flatten_subtree(kid, xf, writer)

def flatten(obj, parent_xf, static_writer, anim_parts):
    """
    Flatten obj tree:
    - If obj.name is in ANIM_NAMES, divert it into a new ObjWriter in anim_parts.
    - Otherwise, add to static_writer and recurse into children.
    """
    xf = compose(parent_xf, obj["loc"], obj["rot"])
    name = obj.get("name", "")

    if name in ANIM_NAMES:
        pw = ObjWriter()
        _flatten_subtree(obj, parent_xf, pw)  # pass PARENT xf so compose happens inside
        # Actually: we want obj's vertices at xf. _flatten_subtree starts by composing,
        # so pass parent_xf (not xf) to let _flatten_subtree compute xf itself.
        # pivot = world pos of the object's local origin in AC coords
        ax, ay, az = xf[0]
        pivot_gl = list(ac_to_gl(ax, ay, az))
        anim_parts.append({"name": name, "writer": pw, "pivot_gl": pivot_gl})
        return

    _add_surfs(obj, xf, static_writer)
    for kid in obj["kids"]:
        flatten(kid, xf, static_writer, anim_parts)


# ── Axis computation ──────────────────────────────────────────────────────────

def compute_axis(name, gl_verts):
    """Determine rotation axis for a part based on its name and geometry."""
    # Hardcoded overrides
    if name in ("fan.l", "fan.r"):
        return [0.0, 0.0, 1.0]   # spin around forward/aft axis
    if name == "rudder":
        return [0.0, 1.0, 0.0]   # vertical fin → Y axis
    if name in ("gear.l", "gear.r",
                "geardoor.b.l", "geardoor.b.r",
                "geardoor.f.l", "geardoor.f.r"):
        return [0.0, 0.0, 1.0]   # main gear folds fore/aft
    if name in ("gear.n", "tire.n"):
        return [1.0, 0.0, 0.0]   # nose gear folds sideways

    # Auto-detect: largest extent axis
    if not gl_verts:
        return [1.0, 0.0, 0.0]
    xs = [v[0] for v in gl_verts]
    ys = [v[1] for v in gl_verts]
    zs = [v[2] for v in gl_verts]
    dx = max(xs) - min(xs); dy = max(ys) - min(ys); dz = max(zs) - min(zs)
    if dx >= dy and dx >= dz:   return [1.0, 0.0, 0.0]
    elif dy >= dx and dy >= dz: return [0.0, 1.0, 0.0]
    else:                        return [0.0, 0.0, 1.0]


# ── OBJ export ────────────────────────────────────────────────────────────────

def write_verts(f, writer):
    for x, y, z in writer.verts:
        gx, gy, gz = ac_to_gl(x, y, z)
        f.write(f"v {gx:.6f} {gy:.6f} {gz:.6f}\n")
    f.write("\n")
    for u, v in writer.uvs:
        f.write(f"vt {u:.6f} {v:.6f}\n")
    f.write("\n")
    for nx, ny, nz in writer.normals:
        gnx, gny, gnz = ac_to_gl(nx, ny, nz)
        f.write(f"vn {gnx:.6f} {gny:.6f} {gnz:.6f}\n")
    f.write("\n")
    for mat_idx in sorted(writer.groups.keys()):
        f.write(f"usemtl mat{mat_idx}\n")
        for tri in writer.groups[mat_idx]:
            refs = " ".join(f"{v+1}/{vt+1}/{vn+1}" for v, vt, vn in tri)
            f.write(f"f {refs}\n")
        f.write("\n")

def write_static_obj(writer, materials, obj_path):
    mtl_path = obj_path.replace(".obj", ".mtl")
    mtl_name = os.path.basename(mtl_path)

    with open(mtl_path, "w") as f:
        for i, mat in enumerate(materials):
            f.write(f"newmtl mat{i}\n")
            r, g, b = mat.get("rgb", (1,1,1))
            f.write(f"Kd {r:.4f} {g:.4f} {b:.4f}\n")
            ar, ag, ab = mat.get("amb", (0.2,0.2,0.2))
            f.write(f"Ka {ar:.4f} {ag:.4f} {ab:.4f}\n")
            f.write(f"d {1.0 - mat.get('trans', 0.0):.4f}\n\n")

    with open(obj_path, "w") as f:
        f.write(f"mtllib {mtl_name}\n\n")
        write_verts(f, writer)

    print(f"Static:  {obj_path}  ({len(writer.verts)} verts, {writer.tri_count()} tris)")

def write_part_obj(writer, obj_path, mtl_name):
    with open(obj_path, "w") as f:
        f.write(f"mtllib {mtl_name}\n\n")
        write_verts(f, writer)


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python ac2obj.py input.ac output.obj")
        sys.exit(1)

    ac_path  = sys.argv[1]
    obj_path = sys.argv[2]
    out_dir  = os.path.dirname(os.path.abspath(obj_path))
    mtl_name = os.path.basename(obj_path).replace(".obj", ".mtl")
    base     = os.path.splitext(os.path.basename(obj_path))[0]  # e.g. "erj195"

    print(f"Parsing {ac_path} ...")
    materials, roots = parse_ac(ac_path)
    print(f"  {len(materials)} materials, {len(roots)} root objects")

    static_writer = ObjWriter()
    anim_parts    = []
    identity_xf   = ((0,0,0), [1,0,0, 0,1,0, 0,0,1])

    for root in roots:
        flatten(root, identity_xf, static_writer, anim_parts)

    write_static_obj(static_writer, materials, obj_path)

    parts_json = {"parts": []}

    for part in anim_parts:
        name   = part["name"]
        pw     = part["writer"]
        pivot  = part["pivot_gl"]
        gl_v   = pw.gl_verts()
        axis   = compute_axis(name, gl_v)
        safe   = name.replace(".", "_")
        pobj   = os.path.join(out_dir, f"{base}_{safe}.obj")
        write_part_obj(pw, pobj, mtl_name)
        parts_json["parts"].append({
            "name":  name,
            "obj":   f"{base}_{safe}.obj",
            "pivot": pivot,
            "axis":  axis,
        })
        print(f"  Part: {name:20s}  pivot={[f'{v:.2f}' for v in pivot]}  axis={axis}  {pw.tri_count()} tris")

    json_path = os.path.join(out_dir, f"{base}_parts.json")
    with open(json_path, "w") as f:
        json.dump(parts_json, f, indent=2)
    print(f"Parts JSON: {json_path}  ({len(parts_json['parts'])} parts)")
