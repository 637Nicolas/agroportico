import numpy as np, json
# ---------- Parámetros del prototipo (supuestos típicos, motor TT 1:48 y celdas 18650) ----------
P=dict(m=1.3,r=0.033,R=7.5,L=0.004,Ke=0.28,Kt=0.17,b=0.002,Jw=2e-5,Crr=0.03,g=9.81,Vmax=6.0,
       Vbat=7.4,Cbat=2.6,I_elec=0.25)
plants=np.array([0.30,0.60,0.90,1.20,1.50,1.80])
dec=['C','A','B','C','A','B']            # decisión simulada por planta
dwell={'A':0.0,'B':1.0,'C':1.8}          # tiempo de aspersión (s)
Kx,Kv,Kiv=3.0,40.0,200.0               # P de posición (1/s); PI de velocidad (V·s/m, V/m)
amax=0.6                                 # m/s² límite de aceleración de la referencia
vlim=0.30                                # m/s (límite de velocidad de crucero)
dt=0.001
CRR=[0.03]
def f(s,u):
    x,v,i=s;w=v/P['r']
    Jeq=P['Jw']+P['m']*P['r']**2/4
    tl=CRR[0]*P['m']*P['g']*P['r']/4*np.tanh(v/0.005)
    di=(u-P['R']*i-P['Ke']*w)/P['L']
    dw=(P['Kt']*i-P['b']*w-tl)/Jeq
    return np.array([v,dw*P['r'],di])
def rk4(s,u):
    k1=f(s,u);k2=f(s+dt/2*k1,u);k3=f(s+dt/2*k2,u);k4=f(s+dt*k3,u);return s+dt/6*(k1+2*k2+2*k3+k4)
rng0=np.random.default_rng(3)
seg_crr=rng0.uniform(0.02,0.07,len(plants))   # resistencia a la rodadura variable por tramo (terreno)
meas=rng0.normal(0,0.004,len(plants))        # error de la cámara al ubicar el centro de la planta (σ = 4 mm)
s=np.array([0.0,0.0,0.0]);t=0;k=0;integ=0;eprev=None;vref_prev=0.0
log=[];stops=[];phase='move';tstop=0
while k<len(plants):
    CRR[0]=seg_crr[k];tgt=plants[k]+meas[k];e=tgt-s[0]
    if phase=='move':
        # Control en cascada: lazo de posición (P) -> velocidad de referencia; lazo de velocidad (PI + prealimentación) -> voltaje
        vr=float(np.clip(Kx*e,-vlim,vlim))
        vr=float(np.clip(vr,vref_prev-amax*dt,vref_prev+amax*dt)); vref_prev=vr
        ev=vr-s[1]; integ+=ev*dt
        u=Kv*ev+Kiv*integ+P['Ke']*vr/P['r']
        u=float(np.clip(u,-P['Vmax'],P['Vmax']))
        if abs(e)<0.003 and abs(s[1])<0.004:
            phase='spray';tstop=t;stops.append(dict(planta=k+1,x_obj=float(plants[k]),x=float(s[0]),err_mm=float((s[0]-plants[k])*1000),t_llegada=round(t,2),dec=dec[k]));u=0
    else:
        u=0
        if t-tstop>=dwell[dec[k]]+0.3: phase='move';k+=1;integ=0;eprev=None;vref_prev=0.0
    s=rk4(s,u);t+=dt
    log.append((t,s[0],s[1],s[2],u,phase=='spray' and dec[min(k,len(dec)-1)]!='A',k))
    if t>40:break
L=np.array([(a,b,c,d,e,1.0 if g else 0.0,h) for a,b,c,d,e,g,h in log])
T=L[:,0];X=L[:,1];V=L[:,2];I=L[:,3];U=L[:,4];SP=L[:,5]
# energía y autonomía
Ibat=4*np.abs(I)*np.abs(U)/P['Vbat']+P['I_elec']   # aprox. potencia motores referida a batería (driver ideal)
Iavg=float(np.mean(Ibat));aut_h=P['Cbat']/Iavg
# máximo sobrepaso por tramo
res=dict(T=float(T[-1]),stops=stops,vmax=float(V.max()),Imax=float(np.abs(I).max()),Iavg_motor=float(np.mean(np.abs(I))),
         Ibat_avg=Iavg,autonomia_h=aut_h,max_err_mm=float(max(abs(s['err_mm']) for s in stops)))
# ---------- aspersión (diseño a escala real) ----------
K=0.8/np.sqrt(3)  # L/min/sqrt(bar) boquilla tipo 110-02
Pbar=2.5;Q=K*np.sqrt(Pbar)   # L/min por boquilla
rng=np.random.default_rng(7);N=200;runs=2000
sav=[];volsel=[];voluni=[]
for _ in range(runs):
    st=rng.choice(['A','B','C'],size=N,p=[0.60,0.25,0.15])
    vd=rng.uniform(0.05,0.20,size=N)        # volumen de dosel m3
    tC=1.0+8.0*vd                            # s, proporcional al dosel
    vol=np.where(st=='A',0,np.where(st=='B',2*Q/60*0.8,3*Q/60*tC))
    uni=3*Q/60*np.full(N,1.0+8.0*0.125)      # uniforme: todas las plantas con la dosis del dosel promedio
    volsel.append(vol.sum());voluni.append(uni.sum());sav.append(100*(1-vol.sum()/uni.sum()))
sav=np.array(sav)
res.update(Q=float(Q),Pbar=Pbar,sav_mean=float(sav.mean()),sav_p5=float(np.percentile(sav,5)),sav_p95=float(np.percentile(sav,95)),
           vol_sel=float(np.mean(volsel)),vol_uni=float(np.mean(voluni)))
np.save('sav.npy',sav)
json.dump(res,open('res.json','w'),indent=1,ensure_ascii=False)
# exportar trayectoria submuestreada a 30 fps para el video
idx=np.searchsorted(T,np.arange(0,T[-1],1/30))
json.dump(dict(t=T[idx].round(3).tolist(),x=X[idx].round(4).tolist(),v=V[idx].round(4).tolist(),i=I[idx].round(3).tolist(),u=U[idx].round(3).tolist(),sp=SP[idx].tolist(),k=L[idx,6].astype(int).tolist(),plants=plants.tolist(),dec=dec,stops=stops),open('traj.json','w'))
np.save('full.npy',L[::5])
print(json.dumps(res,indent=1,ensure_ascii=False))
