"""
Unitree G1 tosses an apple from hand to hand  (MuJoCo)
======================================================

Run:      python apple_toss.py                  interactive viewer
          python apple_toss.py --record out.mp4 render a video instead (no window needed)
          python apple_toss.py --tosses 20      stop after 20 tosses

What happens each toss
  1. Plan    Projectile motion gives the release velocity that lands the apple in the other hand
             after a chosen time of flight T:   v0 = (p_target - p_release)/T + (g T/2) z
  2. Throw   Inverse kinematics drives the 7-joint arm so the palm accelerates along v0 over a
             fixed stroke and lets go at the release point. The joints are torque-limited PD servos,
             so the hand lags the plan and the apple leaves a little off the planned velocity.
  3. Flight  The apple is a free rigid body under gravity. Nothing guides it.
  4. Catch   The other arm reads the apple's position and velocity, predicts where the parabola will
             come down, and moves its palm there, travelling with the apple at contact to soften it.
  5. Learn   The difference between the release velocity needed and the one achieved is fed back into
             the next throw from that hand (iterative learning control). This removes most of the systematic
             shortfall; a few centimetres of throw-to-throw scatter remain and the catching hand absorbs them.

Modelling notes, stated plainly
  * Grasping is modelled with a soft weld constraint between palm and apple. It switches on only when
    the apple's centre is within CAPTURE of the palm's cradle point and off at the instant of release.
    The fingers open and close for show; they do not carry the load. If the hand is not within
    CAPTURE of the apple's path, the catch is missed and the apple falls.
  * The pelvis is held by a virtual support stand (a weld to the world), as on a lab gantry.
    There is no balance controller: without the stand the robot falls over, so the stand stays.
  * Robot model: Unitree G1 with hands from MuJoCo Menagerie (g1/LICENSE).

Overlays: cyan dots = parabola predicted at release, white line = path actually flown,
          green ring = predicted catch point, yellow arrow = apple velocity.
"""
import os, sys, csv, argparse, time
if sys.platform.startswith("linux") and "--record" in sys.argv: os.environ.setdefault("MUJOCO_GL", "egl")   # headless rendering
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------------------------------
#  Arm kinematics
# ----------------------------------------------------------------------------------------------
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll","wrist_pitch","wrist_yaw"]
FING=["hand_thumb_0","hand_thumb_1","hand_thumb_2","hand_index_0","hand_index_1","hand_middle_0","hand_middle_1"]
UP=np.array([0,0,1.])
class Hand:
    def __init__(s,m,side):
        s.m=m; s.side=side; s.sgn=1 if side=="left" else -1
        jid=lambda n: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f"{side}_{n}_joint")
        aid=lambda n: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,f"{side}_{n}_joint")
        s.aj=[jid(n) for n in ARM]; s.qa=[m.jnt_qposadr[j] for j in s.aj]; s.da=[m.jnt_dofadr[j] for j in s.aj]
        s.aa=[aid(n) for n in ARM]; s.fa=[aid(n) for n in FING]
        s.body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,f"{side}_wrist_yaw_link")
        s.n_loc=np.array([0,-s.sgn,0.])          # palm normal in the hand frame
        s.c_loc=np.array([0.124,-s.sgn*0.048,0.]) # where the apple centre rests in the cupped hand
    def frame(s,d):
        R=d.xmat[s.body].reshape(3,3); return d.xpos[s.body]+R@s.c_loc, R
    def fingers(s,grip):
        """Finger targets. grip = -1 wide open (catching), 0 cupped cradle, 1 closed around the apple.
        Order: thumb0, thumb1, thumb2, index0, index1, middle0, middle1."""
        wide=np.array([0,-0.55,0.0,0.12,0.15,0.12,0.15]); cup=np.array([0,0.15,0.25,0.45,0.55,0.45,0.55]); shut=np.array([0,0.70,1.00,0.95,1.15,0.95,1.15])
        g=float(np.clip(grip,-1,1)); v=cup+(cup-wide)*g if g<0 else cup+(shut-cup)*g
        return s.sgn*np.array([1,1,1,-1,-1,-1,-1])*v
def rot_des(side,n,f):
    """Hand frame with palm normal n and fingers as close to f as possible."""
    n=n/np.linalg.norm(n); f=f-n*(f@n); f/=np.linalg.norm(f)
    y=-n if side=="left" else n
    return np.column_stack([f,y,np.cross(f,y)])
def ik(m,dk,hand,p,R,q0=None,iters=12,posture=None):
    """Damped least-squares IK for one arm on scratch data dk (7 joints, 6-D task + posture null space)."""
    if q0 is not None: dk.qpos[hand.qa]=q0
    jp=np.zeros((3,m.nv)); jr=np.zeros((3,m.nv))
    for _ in range(iters):
        mujoco.mj_kinematics(m,dk); mujoco.mj_comPos(m,dk)
        c,Rc=hand.frame(dk)
        Re=R@Rc.T; w=0.5*np.array([Re[2,1]-Re[1,2],Re[0,2]-Re[2,0],Re[1,0]-Re[0,1]])
        e=np.concatenate([p-c,w])
        mujoco.mj_jac(m,dk,jp,jr,c,hand.body)
        J=np.vstack([jp[:,hand.da],jr[:,hand.da]])
        Jp=J.T@np.linalg.inv(J@J.T+1e-4*np.eye(6))
        dq=Jp@e
        if posture is not None: dq+=(np.eye(7)-Jp@J)@(0.1*(posture-dk.qpos[hand.qa]))
        q=dk.qpos[hand.qa]+np.clip(dq,-0.3,0.3)
        lo=m.jnt_range[hand.aj,0]+0.02; hi=m.jnt_range[hand.aj,1]-0.02
        dk.qpos[hand.qa]=np.clip(q,lo,hi)
    mujoco.mj_kinematics(m,dk); c,_=hand.frame(dk)
    return dk.qpos[hand.qa].copy(), np.linalg.norm(p-c)


# ----------------------------------------------------------------------------------------------
#  Throw / flight / catch controller
# ----------------------------------------------------------------------------------------------
G=9.81
def smooth(x):
    x=min(max(x,0.0),1.0); return x*x*x*(10-15*x+6*x*x)
class TossSim:
    HOME={"left":np.array([0.30,0.17,0.95]),"right":np.array([0.30,-0.17,0.95])}
    POST={"left":np.array([0.0,0.25,0,0.9,0,0,0]),"right":np.array([0.0,-0.25,0,0.9,0,0,0])}
    FWD=np.array([1.0,0,0])
    T_FLIGHT=0.56      # planned time of flight (s)
    L_ACC=0.20         # distance over which the hand accelerates the apple (m)
    T_HOLD,T_WIND,T_SETTLE=0.5,0.45,0.55
    T_BRAKE=0.07       # time taken to stop the throwing hand after release (s)
    CAPTURE=0.04       # the palm grasps when the apple centre is within this distance of the cradle point (m)
    BETA=0.45          # catching hand moves at this fraction of the apple's speed at contact
    def __init__(s,xml="scene_toss.xml"):
        s.m=m=mujoco.MjModel.from_xml_path(xml); s.d=mujoco.MjData(m); s.dk=mujoco.MjData(m)
        s.H={k:Hand(m,k) for k in("left","right")}
        s.aq=m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,"apple_joint")]
        s.av=m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,"apple_joint")]
        s.R=m.geom_size[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"apple"),0]
        s.kdkp={k:(-m.actuator_biasprm[h.aa,2]/m.actuator_gainprm[h.aa,0]) for k,h in s.H.items()}
        s.eq={k:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,k+"_grasp") for k in s.H}
        s.qdes={}; s.cmd={}
        for k,h in s.H.items():
            q,_=ik(m,s.dk,h,s.HOME[k],rot_des(k,UP,s.FWD),q0=s.POST[k],iters=200,posture=s.POST[k])
            s.d.qpos[h.qa]=q; s.qdes[k]=q; s.cmd[k]=dict(p=s.HOME[k].copy(),n=UP.copy(),grip=0.0)
            s.d.ctrl[h.aa]=q; s.d.ctrl[h.fa]=h.fingers(0)
        mujoco.mj_forward(m,s.d)
        s.thrower,s.catcher="right","left"
        s.place_apple(s.thrower)
        s.learn={k:np.zeros(3) for k in s.H}     # learned correction to the commanded release velocity
        s.phase="hold"; s.t0=0.0; s.log=[]; s.trail=[]; s.pred=None; s.n_catch=0; s.n_drop=0; s.rec=None
        s.plan=None
    # ---------- helpers ----------
    @property
    def t(s): return s.d.time
    def apple(s): return s.d.qpos[s.aq:s.aq+3].copy(), s.d.qvel[s.av:s.av+3].copy()
    def place_apple(s,k):
        c,_=s.H[k].frame(s.d); s.d.qpos[s.aq:s.aq+3]=c+np.array([0,0,0.004]); s.d.qpos[s.aq+3:s.aq+7]=[1,0,0,0]; s.d.qvel[s.av:s.av+6]=0
        mujoco.mj_forward(s.m,s.d)
        for j in s.H: s.grasp(j,j==k)
    def grasp(s,k,on):
        """Switch the palm-apple grasp constraint. On: hold the apple at the cradle point, keeping its current spin angle."""
        e=s.eq[k]
        if on:
            h=s.H[k]; qh=np.zeros(4); qa=s.d.qpos[s.aq+3:s.aq+7]; qi=np.zeros(4); qr=np.zeros(4)
            mujoco.mju_mat2Quat(qh,s.d.xmat[h.body]); mujoco.mju_negQuat(qi,qh); mujoco.mju_mulQuat(qr,qi,qa)
            s.m.eq_data[e,0:3]=0; s.m.eq_data[e,3:6]=h.c_loc; s.m.eq_data[e,6:10]=qr
        s.d.eq_active[e]=1 if on else 0
    def holder(s):
        return next((k for k in s.H if s.d.eq_active[s.eq[k]]),None)
    def goto(s,k,p,n,grip):
        s.cmd[k]=dict(p=np.asarray(p,float),n=np.asarray(n,float),grip=grip)
    def new_plan(s):
        """Ballistics: pick the release velocity that lands the apple in the other hand after T_FLIGHT."""
        T,C=s.thrower,s.catcher
        p_rel=s.HOME[T]+np.array([0,-s.H[T].sgn*0.02,0.06])
        p_tgt=s.HOME[C]+np.array([0,0,0.03])
        v0=(p_tgt-p_rel)/s.T_FLIGHT+np.array([0,0,0.5*G*s.T_FLIGHT])
        vc=v0+s.learn[T]; sp=np.linalg.norm(vc); u=vc/sp
        s.plan=dict(p_rel=p_rel,p_tgt=p_tgt,v0=v0,vc=vc,u=u,sp=sp,p_start=p_rel-u*s.L_ACC,
                    t_acc=2*s.L_ACC/sp,a=sp*sp/(2*s.L_ACC),released=False)
    def predict(s,p,v,z):
        """Time and state at which a ballistic apple comes down through height z."""
        disc=v[2]**2+2*G*(p[2]-z)
        if disc<0: return None
        tc=(v[2]+np.sqrt(disc))/G
        return tc, p+v*tc+np.array([0,0,-0.5*G*tc*tc]), v+np.array([0,0,-G*tc])
    def throw_point(s,tau):
        """Hand path for the throw: constant acceleration to the release speed, then a hard brake."""
        P=s.plan; ta=P["t_acc"]; td=s.T_BRAKE
        if tau<=ta: dist=0.5*P["a"]*tau*tau
        elif tau<=ta+td: x=tau-ta; dist=s.L_ACC+P["sp"]*x-0.5*(P["sp"]/td)*x*x
        else: dist=s.L_ACC+0.5*P["sp"]*td
        return P["p_start"]+P["u"]*dist
    # ---------- state machine ----------
    def control(s):
        T,C=s.thrower,s.catcher; tau=s.t-s.t0; pa,va=s.apple(); P=s.plan
        if s.phase=="hold":
            s.goto(T,s.HOME[T],UP,0.35); s.goto(C,s.HOME[C],UP,0.0)
            if tau>s.T_HOLD: s.new_plan(); s.phase="wind"; s.t0=s.t
        elif s.phase=="wind":
            a=smooth(tau/s.T_WIND)
            n=UP*(1-a)+P["u"]*a
            s.goto(T,s.HOME[T]*(1-a)+P["p_start"]*a,n,0.35*(1-a))
            s.goto(C,s.HOME[C]*(1-a)+P["p_tgt"]*a,UP*(1-a)+(P["u"]*np.array([-1,-1,1]))*a,-a)
            if tau>s.T_WIND+0.08: s.phase="throw"; s.t0=s.t
        elif s.phase=="throw":
            ta=P["t_acc"]; P["t_throw"]=s.t0
            s.goto(T,s.throw_point(tau),P["u"],0.35-1.35*smooth((tau-0.45*ta)/(0.45*ta)))   # fingers open just before release
            c,_=s.H[T].frame(s.d)
            if tau>=ta and s.holder()==T: s.grasp(T,False)                                     # let go
            if not P["released"] and s.holder() is None and (pa-c)@P["u"]>0.01:
                P["released"]=True
                pr=s.predict(pa,va,P["p_tgt"][2])
                s.rec=dict(n=len(s.log)+1,frm=T,t_rel=s.t,p_rel=pa,v_rel=va,v_plan=P["v0"],apex_pred=pa[2]+max(va[2],0)**2/(2*G),
                           t_flight_pred=pr[0] if pr else np.nan,land_pred=pr[1] if pr else pa,apex=pa[2],E0=0.5*va@va+G*pa[2],dE=0.0)
                s.pred=(pa,va,s.t); s.trail=[]; s.rec["samples"]=[]; s.rec["tgt"]=P["p_tgt"]
                # iterative learning: compare with the velocity that would have hit the target from here
                v_req=(P["p_tgt"]-pa)/s.T_FLIGHT+np.array([0,0,0.5*G*s.T_FLIGHT])
                if va@P["u"]>0.8*np.linalg.norm(va) and np.linalg.norm(v_req-va)<1.5:
                    s.learn[T]=np.clip(s.learn[T]+0.25*(v_req-va),-1.2,1.2)
                s.phase="flight"; s.t0=s.t
            if tau>ta+0.5: s.fail("no release")
        elif s.phase=="flight":
            tt=s.t-P["t_throw"]; a=smooth((tt-P["t_acc"]-s.T_BRAKE)/0.45)   # finish braking, then drift home
            s.goto(T,s.throw_point(tt)*(1-a)+s.HOME[T]*a,P["u"]*(1-a)+UP*a,a-1)
            r=s.rec; r["apex"]=max(r["apex"],pa[2]); r["dE"]=max(r["dE"],abs(0.5*va@va+G*pa[2]-r["E0"]))
            if s.holder() is None: s.trail.append(pa); r["samples"].append((s.t-r["t_rel"],*pa))
            zc=P["p_tgt"][2]; pr=s.predict(pa,va,zc)
            cC,_=s.H[C].frame(s.d)
            if pr is not None and pr[0]>0.004:
                tc,pc,vc=pr; s.catch=dict(tc=s.t+tc,pc=pc,vc=vc)
            K=s.catch; dt=s.t-K["tc"]; sp=np.linalg.norm(K["vc"]); u=K["vc"]/sp
            pc=K["pc"].copy(); pc[:2]=np.clip(pc[:2],s.HOME[C][:2]-0.16,s.HOME[C][:2]+0.16)
            Tpre,Tpost=0.10,0.16; vh=s.BETA*sp
            if dt<-Tpre: off=-0.5*vh*Tpre
            elif dt<0: x=dt+Tpre; off=-0.5*vh*Tpre+0.5*(vh/Tpre)*x*x
            elif dt<Tpost: off=vh*dt-0.5*(vh/Tpost)*dt*dt
            else: off=0.5*vh*Tpost
            if s.holder() is None and np.linalg.norm(pa-cC)<s.CAPTURE and va@u>0: s.grasp(C,True); r["t_catch"]=s.t; r["t_flight_pred"]=s.predict(r["p_rel"],r["v_rel"],pa[2])[0]; e=pa-cC; r["lateral"]=np.linalg.norm(e-u*(e@u))
            s.goto(C,pc+u*off,-u,0.75 if s.holder()==C else -1.0)
            if dt>Tpost+0.05:
                ok=s.holder()==C
                r.update(t_flight=K["tc"]-r["t_rel"],land=K["pc"],caught=ok)
                if "t_catch" in r: r["t_air"]=r["t_catch"]-r["t_rel"]; s.log.append(r)
                if ok: s.n_catch+=1; s.phase="settle"; s.t0=s.t; s.p_from=pc+u*off; s.n_from=-u
                else: s.fail("missed")
            elif pa[2]<0.6: r.update(t_flight=np.nan,land=pa,caught=False); s.log.append(r); s.fail("dropped")
        elif s.phase=="settle":
            a=smooth(tau/s.T_SETTLE)
            s.goto(C,s.p_from*(1-a)+s.HOME[C]*a,s.n_from*(1-a)+UP*a,0.75-0.4*a); s.goto(T,s.HOME[T],UP,0.0)
            if tau>s.T_SETTLE:
                cC,_=s.H[C].frame(s.d)
                if s.holder()!=C: s.fail("slipped")
                else: s.thrower,s.catcher=C,T; s.phase="hold"; s.t0=s.t
        s.apply()
    def fail(s,why):
        s.n_drop+=1; s.last_fail=why
        for k in s.H: s.goto(k,s.HOME[k],UP,0.0); s.grasp(k,False)
        s.phase="recover"; s.t0=s.t
    def apply(s):
        if s.phase=="recover":
            if s.t-s.t0>0.8: s.place_apple(s.thrower); s.phase="hold"; s.t0=s.t
        dt=s.m.opt.timestep
        for k,h in s.H.items():
            c=s.cmd[k]
            q,_=ik(s.m,s.dk,h,c["p"],rot_des(k,c["n"],s.FWD),q0=s.qdes[k],iters=3,posture=s.POST[k])
            qd=np.clip((q-s.qdes[k])/dt,-25,25); s.qdes[k]=q
            s.d.ctrl[h.aa]=np.clip(q+s.kdkp[k]*qd,s.m.actuator_ctrlrange[h.aa,0],s.m.actuator_ctrlrange[h.aa,1])
            s.d.ctrl[h.fa]=h.fingers(c["grip"])
    def step(s):
        s.control(); mujoco.mj_step(s.m,s.d)


# ----------------------------------------------------------------------------------------------
#  Drawing, logging, entry point
# ----------------------------------------------------------------------------------------------
def _geom(scn, gtype, size, pos, rgba, mat=None):
    if scn.ngeom >= scn.maxgeom: return None
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float),
                        (np.eye(3) if mat is None else mat).flatten(), np.asarray(rgba, np.float32))
    scn.ngeom += 1
    return g

def _line(scn, gtype, width, a, b, rgba):
    g = _geom(scn, gtype, [0, 0, 0], [0, 0, 0], rgba)
    if g is not None: mujoco.mjv_connector(g, gtype, width, np.asarray(a, float), np.asarray(b, float))

def draw_overlays(s, scn):
    """Predicted parabola, flown path, catch point and velocity arrow."""
    if s.pred is None or not s.log and s.rec is None: return
    p0, v0, t0 = s.pred
    r = s.rec
    tf = r["t_flight_pred"] if np.isfinite(r["t_flight_pred"]) else 0.6
    for t in np.arange(0.0, tf + 1e-6, 0.03):
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.006, 0, 0], p0 + v0 * t + np.array([0, 0, -0.5 * G * t * t]), [0.35, 0.85, 1.0, 0.9])
    tr = s.trail[::6]
    for a, b in zip(tr[:-1], tr[1:]):
        _line(scn, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.0025, a, b, [1, 1, 1, 0.9])
    _geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.05, 0.0015, 0], r["land_pred"] - np.array([0, 0, s.R]), [0.3, 0.95, 0.5, 0.55])
    if s.phase == "flight" and s.holder() is None:
        pa, va = s.apple()
        _line(scn, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, pa, pa + 0.07 * va, [1.0, 0.82, 0.25, 1])

def toss_line(r):
    sp = np.linalg.norm(r["v_rel"]); ang = np.degrees(np.arctan2(r["v_rel"][2], np.hypot(*r["v_rel"][:2])))
    return (f"{r['n']:>3}  {r['frm']:>5} -> {'left' if r['frm']=='right' else 'right':<5}  {sp:5.2f} m/s  {ang:5.1f} deg   "
            f"{r['t_flight_pred']:.3f} / {r.get('t_air', float('nan')):.3f} s   {r['apex_pred']:.3f} / {r['apex']:.3f} m   "
            f"{1000*r.get('lateral', float('nan')):5.1f} mm   {'caught' if r['caught'] else 'MISSED'}")

HEADER = ("  #  from     to      release   angle     flight pred / meas    apex pred / meas     catch offset   result\n" + "-" * 104)

def save_results(s, outdir):
    """CSV of every toss, plus a figure comparing the flown path with the analytic parabola."""
    if not s.log: return
    with open(os.path.join(outdir, "toss_log.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["toss", "from", "release_speed_m_s", "vx", "vy", "vz", "flight_pred_s", "flight_meas_s", "apex_pred_m", "apex_meas_m",
                    "catch_lateral_offset_mm", "max_energy_drift_J_per_kg", "caught"])
        for r in s.log:
            w.writerow([r["n"], r["frm"], round(float(np.linalg.norm(r["v_rel"])), 4), *np.round(r["v_rel"], 4), round(r["t_flight_pred"], 4),
                        round(r.get("t_air", float("nan")), 4), round(r["apex_pred"], 4), round(r["apex"], 4),
                        round(1000 * r.get("lateral", float("nan")), 2), f"{r['dE']:.2e}", int(r["caught"])])
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.9), dpi=160)
    good = [r for r in s.log if r["caught"] and len(r["samples"]) > 5]
    for r in good[-8:]:
        a = np.array(r["samples"]); t = a[:, 0]; p = a[:, 1:]
        ana = r["p_rel"] + np.outer(t, r["v_rel"]) + np.outer(-0.5 * G * t * t, [0, 0, 1])
        ax[0].plot(ana[:, 1], ana[:, 2], "-", color="#1f5fa8", lw=3, alpha=0.25)
        ax[0].plot(p[::8, 1], p[::8, 2], ".", color="#c0392b", ms=3)
        ax[1].plot(t, 1000 * np.linalg.norm(p - ana, axis=1), color="#444", lw=0.9)
    ax[0].plot([], [], "-", color="#1f5fa8", lw=3, alpha=0.4, label="analytic parabola"); ax[0].plot([], [], ".", color="#c0392b", label="simulated apple")
    ax[0].set_xlabel("y (m), robot's right to left"); ax[0].set_ylabel("height z (m)"); ax[0].set_title("Flight paths, last 8 tosses", fontsize=10); ax[0].legend(fontsize=8, frameon=False); ax[0].set_aspect("equal")
    ax[1].set_xlabel("time since release (s)"); ax[1].set_ylabel("|simulated - analytic| (mm)"); ax[1].set_title("Departure from the parabola (integrator error, about g dt t / 2)", fontsize=9)
    n = [r["n"] for r in s.log]; err = [1000 * np.linalg.norm((r["land_pred"] - r["tgt"])[:2]) for r in s.log]
    ax[2].plot(n, err, "o-", color="#2a7f62", ms=3, lw=1); ax[2].set_xlabel("toss number"); ax[2].set_ylabel("landing error before catch (mm)")
    ax[2].set_title("Landing error of each throw, before the catching hand corrects", fontsize=9)
    for q in ax: q.grid(True, lw=0.3, alpha=0.5)
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "toss_analysis.png")); plt.close(fig)

def hud_lines(s):
    lines = [f"Unitree G1 apple toss     t = {s.t:5.1f} s     caught {s.n_catch} / {s.n_catch + s.n_drop}"]
    r = s.rec
    if r is not None:
        sp = np.linalg.norm(r["v_rel"])
        lines.append(f"toss {r['n']}: {r['frm']} hand, release {sp:.2f} m/s")
        ta = r.get("t_air")
        lines.append(f"time of flight   predicted {r['t_flight_pred']:.3f} s   measured " + (f"{ta:.3f} s" if ta else "..."))
        lines.append(f"apex height      predicted {r['apex_pred']:.3f} m   measured {r['apex']:.3f} m")
    return lines

def main():
    ap = argparse.ArgumentParser(description="Unitree G1 tosses an apple hand to hand in MuJoCo.")
    ap.add_argument("--record", metavar="FILE.mp4", help="render a video instead of opening the viewer")
    ap.add_argument("--tosses", type=int, default=0, help="stop after this many tosses (default: run until closed; 12 when recording)")
    ap.add_argument("--camera", default="three_quarter", help="camera for --record: front or three_quarter")
    a = ap.parse_args()
    s = TossSim(os.path.join(HERE, "scene_toss.xml"))
    print(__doc__.split("Run:")[0].strip()); print(); print(HEADER)
    shown = 0
    def report():
        nonlocal shown
        while shown < len(s.log): print(toss_line(s.log[shown]), flush=True); shown += 1

    if a.record:
        import imageio
        try:
            from PIL import Image, ImageDraw, ImageFont
            try: font = ImageFont.truetype("DejaVuSansMono.ttf", 22)
            except Exception: font = ImageFont.load_default()
        except Exception:
            Image = None
        n = a.tosses or 12; fps = 30
        ren = mujoco.Renderer(s.m, 720, 1280)
        with imageio.get_writer(a.record, fps=fps, quality=8, macro_block_size=None) as wr:
            nxt = 0.0
            while len(s.log) < n and s.t < 5 * n or s.phase in ("flight", "settle"):
                s.step(); report()
                if s.t >= nxt:
                    nxt += 1 / fps
                    ren.update_scene(s.d, a.camera); draw_overlays(s, ren.scene); img = ren.render()
                    if Image is not None:
                        im = Image.fromarray(img); dr = ImageDraw.Draw(im)
                        for i, ln in enumerate(hud_lines(s)): dr.text((24, 20 + 30 * i), ln, fill=(235, 238, 250), font=font)
                        img = np.asarray(im)
                    wr.append_data(img)
        print("\nvideo written to", a.record)
    else:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(s.m, s.d) as v:
            v.cam.lookat[:] = [0.15, 0, 1.05]; v.cam.distance = 2.1; v.cam.azimuth = 155; v.cam.elevation = -12
            t_wall = time.time(); t_sim = s.t
            while v.is_running() and (not a.tosses or len(s.log) < a.tosses or s.phase in ("flight", "settle")):
                target = t_sim + (time.time() - t_wall)
                steps = 0
                while s.t < target and steps < 40: s.step(); steps += 1
                if steps == 40: t_wall = time.time(); t_sim = s.t      # machine slower than real time: do not try to catch up
                report()
                with v.lock():
                    v.user_scn.ngeom = 0; draw_overlays(s, v.user_scn)
                    try: v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "\n".join(hud_lines(s)), ""))
                    except Exception: pass
                v.sync()
                time.sleep(max(0.0, 1 / 60 - 0.002))
    report(); save_results(s, HERE)
    print(f"\n{s.n_catch} caught, {s.n_drop} missed.  Log: toss_log.csv   Figure: toss_analysis.png")

if __name__ == "__main__":
    main()
