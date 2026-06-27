"""
cockpit2obj.py — Convert erj-flightdeck-e2.ac to cockpit.obj/cockpit.mtl
Groups polygons by (material_index, texture_filename).
Coordinate system: GL.x=-AC.z, GL.y=AC.y, GL.z=AC.x
"""

import sys, os, math

AC_PATH  = r"C:\Users\Victor\Desktop\Como as IAs funcionam\tiktok\00001 piloto\E-jet-family-master\Models\FlightDeck\erj-flightdeck-e2.ac"
OUT_DIR  = r"C:\Users\Victor\Desktop\Como as IAs funcionam\tiktok\00001 piloto\cpp\data\models"
OBJ_PATH = os.path.join(OUT_DIR, "cockpit.obj")
MTL_PATH = os.path.join(OUT_DIR, "cockpit.mtl")


# ── AC3D parser ────────────────────────────────────────────────────────────────

def parse_ac(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    if not lines or not lines[0].startswith("AC3D"):
        raise ValueError("Not an AC3D file")

    materials = []
    root_objects = []
    i = [1]

    def peek():
        return lines[i[0]].rstrip() if i[0] < len(lines) else ""

    def next_line():
        while i[0] < len(lines):
            line = lines[i[0]].rstrip()
            i[0] += 1
            if line.strip():
                return line
        return ""

    def read_material(line):
        parts = line.split()
        mat = {"name": parts[1].strip('"'), "rgb": (1,1,1), "amb": (0.2,0.2,0.2), "trans": 0.0}
        try:
            ri = parts.index("rgb");   mat["rgb"]   = (float(parts[ri+1]), float(parts[ri+2]), float(parts[ri+3]))
            ai = parts.index("amb");   mat["amb"]   = (float(parts[ai+1]), float(parts[ai+2]), float(parts[ai+3]))
            ti = parts.index("trans"); mat["trans"]  = float(parts[ti+1])
        except (ValueError, IndexError):
            pass
        return mat

    def read_object():
        obj = {"name": "", "loc": (0,0,0), "rot": None,
               "texture": None, "texrep": (1,1),
               "verts": [], "surfs": [], "kids": []}
        while i[0] < len(lines):
            line = lines[i[0]].rstrip()
            if not line.strip():
                i[0] += 1
                continue
            key = line.split()[0]
            if key in ("OBJECT", "MATERIAL"):
                break  # let parent handle
            i[0] += 1
            if key == "name":
                obj["name"] = line.split(None, 1)[1].strip().strip('"') if len(line.split(None,1))>1 else ""
            elif key == "loc":
                p = line.split(); obj["loc"] = (float(p[1]), float(p[2]), float(p[3]))
            elif key == "rot":
                p = line.split(); obj["rot"] = [float(x) for x in p[1:10]]
            elif key == "texture":
                obj["texture"] = line.split(None, 1)[1].strip().strip('"') if len(line.split(None,1))>1 else None
            elif key == "texrep":
                p = line.split(); obj["texrep"] = (float(p[1]), float(p[2]))
            elif key == "numvert":
                n = int(line.split()[1])
                for _ in range(n):
                    while i[0] < len(lines) and not lines[i[0]].strip(): i[0] += 1
                    p = lines[i[0]].split(); i[0] += 1
                    obj["verts"].append((float(p[0]), float(p[1]), float(p[2])))
            elif key == "numsurf":
                n = int(line.split()[1])
                for _ in range(n):
                    surf = {"flags": 0, "mat": 0, "refs": []}
                    while i[0] < len(lines):
                        sl = lines[i[0]].rstrip()
                        if not sl.strip(): i[0] += 1; continue
                        sk = sl.split()[0]
                        if sk == "SURF":   surf["flags"] = int(sl.split()[1], 16); i[0] += 1
                        elif sk == "mat":  surf["mat"]   = int(sl.split()[1]); i[0] += 1
                        elif sk == "refs":
                            nr = int(sl.split()[1]); i[0] += 1
                            for _ in range(nr):
                                while i[0] < len(lines) and not lines[i[0]].strip(): i[0] += 1
                                rp = lines[i[0]].split(); i[0] += 1
                                surf["refs"].append((int(rp[0]), float(rp[1]), float(rp[2])))
                            break
                        else:
                            break
                    obj["surfs"].append(surf)
            elif key == "kids":
                n = int(line.split()[1])
                for _ in range(n):
                    # advance to next OBJECT line
                    while i[0] < len(lines) and not lines[i[0]].strip().startswith("OBJECT"):
                        i[0] += 1
                    if i[0] >= len(lines): break
                    i[0] += 1  # skip "OBJECT poly" line
                    obj["kids"].append(read_object())
                break
        return obj

    while i[0] < len(lines):
        line = lines[i[0]].rstrip()
        if not line.strip(): i[0] += 1; continue
        key = line.split()[0] if line.split() else ""
        i[0] += 1
        if key == "MATERIAL":
            materials.append(read_material(line))
        elif key == "OBJECT":
            root_objects.append(read_object())

    return materials, root_objects


# ── Transform helpers ──────────────────────────────────────────────────────────

ID_ROT = [1,0,0, 0,1,0, 0,0,1]

def matmul(a, b):
    r = [0.0]*9
    for row in range(3):
        for col in range(3):
            for k in range(3):
                r[row*3+col] += a[row*3+k] * b[k*3+col]
    return r

def transform_vertex(loc, rot, v):
    ox, oy, oz = loc; x, y, z = v
    return (rot[0]*x + rot[1]*y + rot[2]*z + ox,
            rot[3]*x + rot[4]*y + rot[5]*z + oy,
            rot[6]*x + rot[7]*y + rot[8]*z + oz)

def compose(parent_loc, parent_rot, child_loc, child_rot):
    cx, cy, cz = child_loc
    wx = parent_rot[0]*cx + parent_rot[1]*cy + parent_rot[2]*cz + parent_loc[0]
    wy = parent_rot[3]*cx + parent_rot[4]*cy + parent_rot[5]*cz + parent_loc[1]
    wz = parent_rot[6]*cx + parent_rot[7]*cy + parent_rot[8]*cz + parent_loc[2]
    if child_rot is None:
        child_rot = list(ID_ROT)
    new_rot = matmul(parent_rot, child_rot)
    return (wx, wy, wz), new_rot

def ac_to_gl(x, y, z):
    return (-z, y, x)

def flat_normal(pts):
    if len(pts) < 3: return (0,1,0)
    ax = pts[1][0]-pts[0][0]; ay = pts[1][1]-pts[0][1]; az = pts[1][2]-pts[0][2]
    bx = pts[2][0]-pts[0][0]; by = pts[2][1]-pts[0][1]; bz = pts[2][2]-pts[0][2]
    nx = ay*bz - az*by; ny = az*bx - ax*bz; nz = ax*by - ay*bx
    l = math.sqrt(nx*nx+ny*ny+nz*nz)
    return (nx/l, ny/l, nz/l) if l > 1e-10 else (0,1,0)


# ── Geometry collector ─────────────────────────────────────────────────────────

# groups[(mat_idx, tex)] = list of triangles, each = [(gl_pos, (u,v), gl_nrm), ...]
groups = {}
all_verts  = []  # (gl_x, gl_y, gl_z)
all_uvs    = []  # (u, v)
all_normals = [] # (gl_x, gl_y, gl_z)

def collect(obj, parent_loc, parent_rot, parent_tex, parent_texrep):
    loc, rot = compose(parent_loc, parent_rot, obj["loc"], obj["rot"])
    tex    = obj["texture"] if obj["texture"] is not None else parent_tex
    texrep = obj["texrep"]  if obj["texture"] is not None else parent_texrep

    if obj["verts"] and obj["surfs"]:
        world_verts = [transform_vertex(loc, rot, v) for v in obj["verts"]]
        for surf in obj["surfs"]:
            flags = surf["flags"]
            stype = flags & 0x0F
            if stype in (1, 2): continue  # line strips
            refs = surf["refs"]
            if len(refs) < 3: continue

            mat_idx = surf["mat"]
            key = (mat_idx, tex or "")
            if key not in groups:
                groups[key] = []

            pts  = [ac_to_gl(*world_verts[r[0]]) for r in refs]
            uvs  = [(r[1] * texrep[0], (1.0 - r[2]) * texrep[1]) for r in refs]
            nrm  = ac_to_gl(*flat_normal([world_verts[r[0]] for r in refs]))

            # Fan triangulation
            vi0 = len(all_verts);  vt0 = len(all_uvs);  vn0 = len(all_normals)
            all_verts.extend(pts)
            all_uvs.extend(uvs)
            all_normals.append(nrm)

            for t in range(1, len(pts)-1):
                groups[key].append((vi0, vi0+t, vi0+t+1, vt0, vt0+t, vt0+t+1, vn0))

            if flags & 0x20:  # two-sided: reverse
                vi1 = len(all_verts); vt1 = len(all_uvs)
                all_verts.extend(reversed(pts))
                all_uvs.extend(reversed(uvs))
                nrm_rev = (-nrm[0], -nrm[1], -nrm[2])
                vn1 = len(all_normals)
                all_normals.append(nrm_rev)
                for t in range(1, len(pts)-1):
                    groups[key].append((vi1, vi1+t, vi1+t+1, vt1, vt1+t, vt1+t+1, vn1))

    for kid in obj.get("kids", []):
        collect(kid, loc, rot, tex, texrep)


# ── Main ───────────────────────────────────────────────────────────────────────

print(f"Parsing {AC_PATH} ...")
materials, root_objects = parse_ac(AC_PATH)
print(f"  {len(materials)} materials, {len(root_objects)} root objects")

ID = (0.0, 0.0, 0.0)
for ro in root_objects:
    collect(ro, ID, list(ID_ROT), None, (1,1))

n_tris = sum(len(g) for g in groups.values())
print(f"  {len(all_verts)} verts, {n_tris} triangles, {len(groups)} material groups")

# ── Build material name map ────────────────────────────────────────────────────

def mat_name(mat_idx, tex):
    if tex:
        stem = os.path.splitext(os.path.basename(tex))[0]
        # sanitize
        stem = stem.replace(" ", "_").replace(".", "-")
        return f"mat{mat_idx}_{stem}"
    return f"mat{mat_idx}"

# ── Write MTL ─────────────────────────────────────────────────────────────────

print(f"Writing {MTL_PATH} ...")
seen_mats = set()
with open(MTL_PATH, "w") as f:
    f.write("# cockpit.mtl — generated by cockpit2obj.py\n\n")
    for (mat_idx, tex) in sorted(groups.keys()):
        mname = mat_name(mat_idx, tex)
        if mname in seen_mats: continue
        seen_mats.add(mname)
        m = materials[mat_idx] if mat_idx < len(materials) else {"rgb":(1,1,1),"amb":(0.2,0.2,0.2),"trans":0.0}
        r, g, b = m["rgb"]
        f.write(f"newmtl {mname}\n")
        f.write(f"Kd {r:.4f} {g:.4f} {b:.4f}\n")
        ar, ag, ab = m.get("amb", (0.2,0.2,0.2))
        f.write(f"Ka {ar:.4f} {ag:.4f} {ab:.4f}\n")
        f.write(f"d {1.0 - m.get('trans',0):.4f}\n")
        if tex:
            f.write(f"map_Kd {os.path.basename(tex)}\n")
        f.write("\n")

# ── Write OBJ ─────────────────────────────────────────────────────────────────

print(f"Writing {OBJ_PATH} ...")
mtl_name = os.path.basename(MTL_PATH)
with open(OBJ_PATH, "w") as f:
    f.write(f"# cockpit.obj — E195-E2 FlightDeck — generated by cockpit2obj.py\n")
    f.write(f"mtllib {mtl_name}\n\n")
    for gx, gy, gz in all_verts:
        f.write(f"v {gx:.6f} {gy:.6f} {gz:.6f}\n")
    f.write("\n")
    for u, v in all_uvs:
        f.write(f"vt {u:.6f} {v:.6f}\n")
    f.write("\n")
    for nx, ny, nz in all_normals:
        f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
    f.write("\n")
    for (mat_idx, tex) in sorted(groups.keys()):
        mname = mat_name(mat_idx, tex)
        f.write(f"usemtl {mname}\n")
        tris = groups[(mat_idx, tex)]
        for (v0,v1,v2, vt0,vt1,vt2, vn) in tris:
            f.write(f"f {v0+1}/{vt0+1}/{vn+1} {v1+1}/{vt1+1}/{vn+1} {v2+1}/{vt2+1}/{vn+1}\n")
        f.write("\n")

print("Done.")
