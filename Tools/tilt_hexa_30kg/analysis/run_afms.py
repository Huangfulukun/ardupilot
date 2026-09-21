#!/usr/bin/env python3
"""Offline AFMS projections for the 30-kg TiltHexa paper.

AFMS is analysis-only in Paper 1.  This script intentionally uses static
magnitude/geometry constraints (not one-step rate constraints), matching the
paper definition.  It consumes E1 trim results and writes CSV boundaries plus
paper-ready PNG/PDF figures.
"""

import csv
import json
import math
import os
import sys

import numpy as np
import yaml
from scipy.optimize import linprog

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CFG_PATH = os.path.join(ROOT, "config", "tilt_hexa_30kg_seed.yaml")
E1_DIR = os.path.join(ROOT, "results", "E1")
OUT_DIR = os.path.join(E1_DIR, "afms")

PSI_DEG = [90.0, -90.0, -30.0, 150.0, 30.0, -150.0]
SPIN_SIGNS = [+1.0, -1.0, +1.0, -1.0, -1.0, +1.0]


def load_cfg():
    with open(CFG_PATH, "r") as f:
        return yaml.safe_load(f)


def load_trim():
    path = os.path.join(E1_DIR, "trim_sweep_plant_full.csv")
    if not os.path.exists(path):
        raise FileNotFoundError("Run experiments/run_e1_trim.py first")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError("trim_sweep_plant_full.csv is empty")
    return rows


def controller_B(V, cfg):
    geo = cfg["geometry"]
    prop = cfg["propulsion"]
    surf = cfg["aero"]["surfaces"]
    rho = cfg["flight"]["rho_kg_m3"]
    S = geo["wing_area_m2"]; b = geo["wing_span_m"]; c = geo["mean_aero_chord_m"]
    L = geo["arm_radius_m"]; z = geo["rotor_z_m"]; kq = prop["kappa_Q"]

    B = np.zeros((5, 16), dtype=float)
    for i, (psi_d, si) in enumerate(zip(PSI_DEG, SPIN_SIGNS)):
        psi = math.radians(psi_d)
        x = L * math.cos(psi); y = L * math.sin(psi)
        col = 2 * i
        B[:, col:col+2] = np.array([
            [1.0, 0.0],
            [0.0, -1.0],
            [si*kq, -y],
            [z, x],
            [-y, -si*kq],
        ])

    q = 0.5 * rho * V * V
    qS = q*S; qSb = qS*b; qSc = qS*c
    # Match AP_TiltHexa_Effectiveness: surfaces are moment effectors only.
    B[2,12] = +qSb*surf["Cl_da"]; B[2,13] = -qSb*surf["Cl_da"]
    B[3,12] = +qSc*surf["Cm_da"]; B[3,13] = +qSc*surf["Cm_da"]
    B[4,12] = +qSb*surf["Cn_da"]; B[4,13] = -qSb*surf["Cn_da"]
    B[2,14] = +qSb*surf["Cl_drv"]; B[2,15] = -qSb*surf["Cl_drv"]
    B[3,14] = +qSc*surf["Cm_drv"]; B[3,15] = +qSc*surf["Cm_drv"]
    B[4,14] = -qSb*surf["Cn_drv"]; B[4,15] = +qSb*surf["Cn_drv"]
    return B


def static_constraints(cfg):
    N = int(cfg["allocation"]["polygon_facets"])
    Tmax = float(cfg["propulsion"]["max_static_thrust_N"])
    bmin = math.radians(cfg["tilt"]["min_deg"])
    bmax = math.radians(cfg["tilt"]["max_deg"])
    nvar = 16
    A=[]; h=[]
    # Inscribed thrust polygon and exact angular sector.
    R = Tmax * math.cos(math.pi/N)
    for m in range(6):
        c0=2*m
        for j in range(N):
            th = 2*math.pi*j/N + math.pi/N
            row=np.zeros(nvar); row[c0]=math.cos(th); row[c0+1]=math.sin(th)
            A.append(row); h.append(R)
        row=np.zeros(nvar); row[c0]=-math.cos(bmin); row[c0+1]=math.sin(bmin)
        A.append(row); h.append(0.0)
        row=np.zeros(nvar); row[c0]=math.cos(bmax); row[c0+1]=-math.sin(bmax)
        A.append(row); h.append(0.0)

    amax=math.radians(cfg["surfaces"]["aileron_left_max_deg"])
    rvmax=math.radians(cfg["surfaces"]["ruddervator_left_max_deg"])
    for col,lim in zip([12,13,14,15],[amax,amax,rvmax,rvmax]):
        row=np.zeros(nvar); row[col]=1; A.append(row); h.append(lim)
        row=np.zeros(nvar); row[col]=-1; A.append(row); h.append(lim)
    return np.asarray(A), np.asarray(h)


def support_boundary(B, A, h, axes, ntheta=181):
    pts=[]
    for theta in np.linspace(0.0, 2.0*math.pi, ntheta, endpoint=False):
        d2=np.array([math.cos(theta), math.sin(theta)])
        d5=np.zeros(5); d5[axes[0]]=d2[0]; d5[axes[1]]=d2[1]
        c=-(B.T @ d5)
        res=linprog(c, A_ub=A, b_ub=h, bounds=[(None,None)]*16, method="highs")
        if not res.success:
            raise RuntimeError(f"AFMS LP failed at theta={theta}: {res.message}")
        w=B@res.x
        pts.append((w[axes[0]], w[axes[1]], theta))
    return np.asarray(pts)


def choose_states(rows):
    ok=[r for r in rows if int(float(r.get("solver_success", "0")))==1]
    if not ok:
        raise RuntimeError("No successful trim points")
    def V(r): return float(r["V"])
    hover=min(ok,key=V)
    cruise=max(ok,key=V)
    early=min(ok,key=lambda r:abs(V(r)-8.0))
    mixed=[r for r in ok if 0.05 <= float(r.get("gamma_A",0)) <= 0.95]
    if mixed:
        weakest=min(mixed,key=lambda r:float(r["sigma_min"]))
    else:
        interior=[r for r in ok if V(r)>1.0 and V(r)<V(cruise)]
        weakest=min(interior or ok,key=lambda r:float(r["sigma_min"]))
    return [("hover",hover),("early",early),("weakest",weakest),("cruise",cruise)]


def main():
    os.makedirs(OUT_DIR,exist_ok=True)
    cfg=load_cfg(); rows=load_trim(); states=choose_states(rows)
    A,h=static_constraints(cfg)

    specs=[("FxFz",(0,1),"Fx (N)","Fz (N)"),
           ("MxMy",(2,3),"Mx (N m)","My (N m)"),
           ("MxMz",(2,4),"Mx (N m)","Mz (N m)")]

    summary={"seed_status":cfg["metadata"]["status"],"states":{}}
    boundaries={}
    for label,row in states:
        V=float(row["V"]); B=controller_B(V,cfg)
        summary["states"][label]={"V_mps":V,"sigma_min":float(row["sigma_min"])}
        for name,axes,_,_ in specs:
            pts=support_boundary(B,A,h,axes)
            boundaries[(label,name)]=pts
            path=os.path.join(OUT_DIR,f"afms_{name}_{label}.csv")
            np.savetxt(path,pts,delimiter=",",header="x,y,theta_rad",comments="")

    with open(os.path.join(OUT_DIR,"afms_summary.json"),"w") as f:
        json.dump(summary,f,indent=2)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for name,axes,xlab,ylab in specs:
            fig,ax=plt.subplots(figsize=(6.4,5.0))
            for label,_ in states:
                pts=boundaries[(label,name)]
                loop=np.vstack([pts[:,:2],pts[0,:2]])
                ax.plot(loop[:,0],loop[:,1],label=f"{label} ({summary['states'][label]['V_mps']:.0f} m/s)")
            ax.set_xlabel(xlab); ax.set_ylabel(ylab); ax.grid(True,alpha=0.3); ax.legend()
            if cfg["metadata"]["status"]=="REFERENCE_SEED_NOT_MEASURED":
                ax.set_title("REFERENCE SEED MODEL — NOT MEASURED")
            fig.tight_layout()
            fig.savefig(os.path.join(OUT_DIR,f"afms_{name}.png"),dpi=300)
            fig.savefig(os.path.join(OUT_DIR,f"afms_{name}.pdf"))
            plt.close(fig)
    except ImportError:
        print("[AFMS] matplotlib unavailable; CSVs still generated")

    print(json.dumps(summary,indent=2))
    return 0


if __name__=="__main__":
    sys.exit(main())
