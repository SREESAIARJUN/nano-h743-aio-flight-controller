"""Minimal KiCad S-expression reader shared by the Rev 8 analysis and verifier.

parse(text) -> nested lists; atoms are str (quoted strings unquoted).
Helpers find children by head symbol.  No pcbnew dependency.
"""
import re

_TOK = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+')


def parse(text):
    stack = [[]]
    for m in _TOK.finditer(text):
        t = m.group(0)
        if t == "(":
            stack.append([])
        elif t == ")":
            node = stack.pop()
            stack[-1].append(node)
        elif t.startswith('"'):
            stack[-1].append(t[1:-1].replace('\\"', '"'))
        else:
            stack[-1].append(t)
    return stack[0][0]


def kids(node, head):
    return [c for c in node if isinstance(c, list) and c and c[0] == head]


def kid(node, head):
    k = kids(node, head)
    return k[0] if k else None


def val(node, head, idx=1, cast=str):
    k = kid(node, head)
    return cast(k[idx]) if k is not None and len(k) > idx else None


def load(path):
    with open(path) as f:
        return parse(f.read())


def stackup(board):
    st = kid(kid(board, "setup"), "stackup")
    out = []
    for lay in kids(st, "layer"):
        out.append({"name": lay[1], "type": val(lay, "type"),
                    "thickness": val(lay, "thickness", cast=float),
                    "epsilon_r": val(lay, "epsilon_r", cast=float),
                    "material": val(lay, "material")})
    return out


def footprints(board):
    res = []
    for fp in kids(board, "footprint"):
        props = {p[1]: p[2] for p in kids(fp, "property")}
        at = kid(fp, "at")
        pads = []
        for p in kids(fp, "pad"):
            pat = kid(p, "at")
            pads.append({"num": p[1], "type": p[2], "shape": p[3],
                         "at": (float(pat[1]), float(pat[2])),
                         "rot": float(pat[3]) if len(pat) > 3 else 0.0,
                         "size": tuple(float(x) for x in kid(p, "size")[1:3]),
                         "drill": val(p, "drill", cast=float),
                         "layers": kid(p, "layers")[1:],
                         "net": (kid(p, "net")[2] if kid(p, "net") else ""),
                         "zone_connect": val(p, "zone_connect", cast=int)})
        res.append({"lib": fp[1], "layer": val(fp, "layer"), "ref": props.get("Reference"),
                    "props": props, "at": (float(at[1]), float(at[2])),
                    "rot": float(at[3]) if len(at) > 3 else 0.0,
                    "zone_connect": val(fp, "zone_connect", cast=int), "pads": pads, "raw": fp})
    return res
