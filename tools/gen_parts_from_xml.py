"""
Gera erj195_parts.json com pivôs e eixos exatos extraídos do ERJ-195-Model.xml.

Conversão de coordenadas:
  XML.x = fuselagem (nariz=-X, cauda=+X)  →  GL.z = XML.x
  XML.y = lateral   (porto=-Y, boreste=+Y) →  GL.x = XML.y
  XML.z = vertical  (cima=+Z)              →  GL.y = XML.z

  GL(x,y,z) = (XML.y, XML.z, XML.x)
"""

import json, math, os

OUT = r"C:\Users\Victor\Desktop\Como as IAs funcionam\tiktok\00001 piloto\cpp\data\models\erj195_parts.json"

def to_gl(xv, yv, zv):
    return [yv, zv, xv]

def norm(v):
    x, y, z = v
    l = math.sqrt(x*x + y*y + z*z)
    if l < 1e-9: return [1, 0, 0]
    return [x/l, y/l, z/l]

def axis_from_two_pts(x1,y1,z1, x2,y2,z2):
    """Eixo normalizado de dois pontos na linha de charneira (espaço XML → GL)."""
    d = to_gl(x2-x1, y2-y1, z2-z1)
    return norm(d)

def midpoint(x1,y1,z1, x2,y2,z2):
    return to_gl((x1+x2)/2, (y1+y2)/2, (z1+z2)/2)

# ── Definições exatas do XML ───────────────────────────────────────────────────
# Cada entrada: (nome_ac3d, pivot_GL, axis_GL, max_deg, signo, source)
# signo: +1 ou -1 — ajuste fino por inspeção visual

PARTS = []

def add(name, pivot, axis, max_deg=None, sign=1):
    PARTS.append({
        "name": name,
        "pivot": [round(v, 4) for v in pivot],
        "axis":  [round(v, 4) for v in axis],
        "max_deg": max_deg,
        "sign": sign,
    })

# ── Leme ──────────────────────────────────────────────────────────────────────
add("rudder",
    pivot = midpoint(16.1741, 0, 2.5407,  18.7162, 0, 8.2472),
    axis  = axis_from_two_pts(16.1741, 0, 2.5407,  18.7162, 0, 8.2472),
    max_deg = 35, sign = 1)

# ── Elevadores ────────────────────────────────────────────────────────────────
add("elevator.l",
    pivot = midpoint(17.294, -0.99, 1.531,  18.73, -5.22, 1.9713),
    axis  = axis_from_two_pts(17.294, -0.99, 1.531,  18.73, -5.22, 1.9713),
    max_deg = 30, sign = -1)   # FG: ind=-1→dep=30° (cima = trailing edge up)

add("elevator.r",
    pivot = midpoint(17.294,  0.99, 1.531,  18.73,  5.22, 1.9713),
    axis  = axis_from_two_pts(17.294,  0.99, 1.531,  18.73,  5.22, 1.9713),
    max_deg = 30, sign =  1)   # FG: ind=-1→dep=-30° (simétrico)

# ── Ailerons ──────────────────────────────────────────────────────────────────
add("aileron.l",
    pivot = midpoint(2.75, -11.97, 0.72,  1.97, -9.42, 0.50),
    axis  = axis_from_two_pts(2.75, -11.97, 0.72,  1.97, -9.42, 0.50),
    max_deg = 35, sign = 1)

add("aileron.r",
    pivot = midpoint(2.75, 11.97, 0.72,  1.97, 9.42, 0.50),
    axis  = axis_from_two_pts(2.75, 11.97, 0.72,  1.97, 9.42, 0.50),
    max_deg = 35, sign = -1)

# ── Flaps ─────────────────────────────────────────────────────────────────────
add("flap.l.1",
    pivot = midpoint(0.42, -1.70, -0.29,  0.42, -4.14, 0.01),
    axis  = axis_from_two_pts(0.42, -1.70, -0.29,  0.42, -4.14, 0.01),
    max_deg = 25, sign = 1)   # FG: ind=0.45→dep=-25° = fora / baixo

add("flap.l.2",
    pivot = midpoint(0.54, -4.43, 0.03,  1.91, -9.20, 0.48),
    axis  = axis_from_two_pts(0.54, -4.43, 0.03,  1.91, -9.20, 0.48),
    max_deg = 25, sign = 1)

add("flap.r.1",
    pivot = midpoint(0.42,  1.70, -0.29,  0.42,  4.14, 0.01),
    axis  = axis_from_two_pts(0.42,  1.70, -0.29,  0.42,  4.14, 0.01),
    max_deg = 25, sign = -1)  # FG: ind=0.45→dep=+25° (sinal trocado para boreste)

add("flap.r.2",
    pivot = midpoint(0.54,  4.43, 0.03,  1.91,  9.20, 0.48),
    axis  = axis_from_two_pts(0.54,  4.43, 0.03,  1.91,  9.20, 0.48),
    max_deg = 25, sign = -1)

# ── Spoilers ──────────────────────────────────────────────────────────────────
add("spoiler.l.1",
    pivot = midpoint(-0.03, -1.82, -0.13,  -0.03, -2.98, 0.02),
    axis  = axis_from_two_pts(-0.03, -1.82, -0.13,  -0.03, -2.98, 0.02),
    max_deg = 55, sign = 1)

add("spoiler.r.1",
    pivot = midpoint(-0.03,  1.82, -0.13,  -0.03,  2.98, 0.02),
    axis  = axis_from_two_pts(-0.03,  1.82, -0.13,  -0.03,  2.98, 0.02),
    max_deg = 55, sign = -1)

add("spoiler.l.2",
    pivot = midpoint(0.25, -4.57, 0.14,  0.99, -7.13, 0.37),
    axis  = axis_from_two_pts(0.25, -4.57, 0.14,  0.99, -7.13, 0.37),
    max_deg = 50, sign = 1)

add("spoiler.r.2",
    pivot = midpoint(0.25,  4.57, 0.14,  0.99,  7.13, 0.37),
    axis  = axis_from_two_pts(0.25,  4.57, 0.14,  0.99,  7.13, 0.37),
    max_deg = 50, sign = -1)

# ── Trem de nariz ─────────────────────────────────────────────────────────────
nose_pivot = to_gl(-13.60, 0.0, -0.84)    # center do XML
add("gear.n",   pivot=nose_pivot, axis=to_gl(0,1,0),   max_deg=115, sign=1)
add("tire.n",   pivot=nose_pivot, axis=to_gl(0,1,0),   max_deg=115, sign=1)

# Portinholas de nariz — abrem durante transição (pos 0→0.2 = -90°, fecham 0.8→1.0)
nf_pivot_l = midpoint(-15.87, -0.32, -0.83,  -13.96, -0.39, -0.93)
nf_axis_l  = axis_from_two_pts(-15.87, -0.32, -0.83,  -13.96, -0.39, -0.93)
add("geardoor.f.l", pivot=nf_pivot_l, axis=nf_axis_l, max_deg=90, sign=-1)

nf_pivot_r = midpoint(-15.87,  0.32, -0.83,  -13.96,  0.39, -0.93)
nf_axis_r  = axis_from_two_pts(-15.87,  0.32, -0.83,  -13.96,  0.39, -0.93)
add("geardoor.f.r", pivot=nf_pivot_r, axis=nf_axis_r, max_deg=90, sign= 1)

nb_pivot_l = midpoint(-13.96, -0.30, -0.95,  -13.43, -0.30, -0.96)
nb_axis_l  = axis_from_two_pts(-13.96, -0.30, -0.95,  -13.43, -0.30, -0.96)
add("geardoor.b.l", pivot=nb_pivot_l, axis=nb_axis_l, max_deg=50, sign=-1)

nb_pivot_r = midpoint(-13.96,  0.30, -0.95,  -13.43,  0.30, -0.96)
nb_axis_r  = axis_from_two_pts(-13.96,  0.30, -0.95,  -13.43,  0.30, -0.96)
add("geardoor.b.r", pivot=nb_pivot_r, axis=nb_axis_r, max_deg=50, sign= 1)

# ── Trem principal ────────────────────────────────────────────────────────────
main_l_pivot = to_gl(0, -2.53, -0.37)
add("gear.l",  pivot=main_l_pivot, axis=to_gl(1,0,0),   max_deg=75, sign=1)
add("tire.l",  pivot=main_l_pivot, axis=to_gl(1,0,0),   max_deg=75, sign=1)

main_r_pivot = to_gl(0,  2.53, -0.37)
add("gear.r",  pivot=main_r_pivot, axis=to_gl(-1,0,0),  max_deg=75, sign=1)
add("tire.r",  pivot=main_r_pivot, axis=to_gl(-1,0,0),  max_deg=75, sign=1)

# ── Fans ──────────────────────────────────────────────────────────────────────
add("fan.l",
    pivot = to_gl(0, -4.04, -1.08),
    axis  = to_gl(-1, 0, 0),   # XML axis=(-1,0,0)
    max_deg = None, sign = 1)

add("fan.r",
    pivot = to_gl(0,  4.04, -1.08),
    axis  = to_gl( 1, 0, 0),   # XML axis=(+1,0,0)
    max_deg = None, sign = 1)

# ── Reversor (translate no XML, mas aqui fazemos rotate simples) ──────────────
add("reverser",
    pivot = to_gl(-4.04, -4.04, -1.06),  # posição aproximada do reversor
    axis  = to_gl(0, 0, 1),
    max_deg = 60, sign = 1)

# ── Gerar JSON ────────────────────────────────────────────────────────────────
# Mantém o campo "obj" baseado no nome (mesmo padrão do ac2obj.py)
BASE = "erj195"

def safe(name):
    return name.replace(".", "_")

with open(OUT) as f:
    existing = json.load(f)

# Mapa nome → obj filename do JSON existente
obj_map = {p["name"]: p["obj"] for p in existing["parts"]}

out_parts = []
for p in PARTS:
    n = p["name"]
    # Usa o obj do converter anterior se existir; senão constrói o nome
    obj_file = obj_map.get(n, f"{BASE}_{safe(n)}.obj")
    out_parts.append({
        "name":    n,
        "obj":     obj_file,
        "pivot":   p["pivot"],
        "axis":    p["axis"],
        "max_deg": p["max_deg"],
        "sign":    p["sign"],
    })

doc = {"parts": out_parts}
with open(OUT, "w") as f:
    json.dump(doc, f, indent=2)

print(f"Escrito: {OUT}")
for p in out_parts:
    print(f"  {p['name']:20s}  pivot={p['pivot']}  axis={p['axis']}")
