"""Pure trajectory math and pilot-owned repeated mode handover; no ROS or flight services."""
import math
import numpy as np

SPEED_XY, SPEED_Z, ACC_XY, ACC_Z = 1., .3, .5, .6

def replan_recoverable(phase, errors):
    """Only transient planning/transport gaps; callers also require fresh inputs."""
    if phase=='PLANNING_FAILED':return set(errors)=={'planner:PLANNING_FAILED'}
    return phase=='ACTIVE' and bool(errors) and set(errors)<= {
        'await matching trajectory','trajectory stale','trajectory not current'}

def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<=1e-9:
        raise ValueError('invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def interpolate_pose(t0,p0,q0,t1,p1,q1,stamp):
    """Interpolate a close time bracket; quaternions may use opposite signs."""
    if not t0<=stamp<=t1 or t1<=t0:raise ValueError('invalid pose time bracket')
    alpha=(stamp-t0)/(t1-t0)
    q0=np.asarray(q0,float);q1=np.asarray(q1,float)
    rotation(q0);rotation(q1)
    if np.dot(q0,q1)<0:q1=-q1
    q=(1-alpha)*q0+alpha*q1;q/=np.linalg.norm(q)
    return (1-alpha)*np.asarray(p0)+alpha*np.asarray(p1),q

class Alignment:
    def __init__(self,lp,lq,fp,fq):
        self.R=rotation(fq)@rotation(lq).T
        self.t=np.asarray(fp)-self.R@np.asarray(lp)
    def point(self,p):return self.R@np.asarray(p)+self.t
    def at_pose(self,lp,fp):
        """Same frozen rotation with the currently measured frame translation."""
        result=object.__new__(Alignment);result.R=self.R.copy()
        result.t=np.asarray(fp,dtype=float)-self.R@np.asarray(lp,dtype=float)
        return result
    def check(self,lp,lq,fp,fq):
        relative=rotation(fq)@(self.R@rotation(lq)).T
        angle=math.acos(np.clip((np.trace(relative)-1)/2,-1,1))
        return np.linalg.norm(self.point(lp)-fp)<=.2 and angle<=math.radians(5)

def extrema(poly,duration):
    poly=np.trim_zeros(np.asarray(poly,float),'f')
    if not len(poly):return [0.,duration]
    roots=np.roots(np.polyder(poly)) if len(poly)>1 else []
    return [0.,duration]+[float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<r.real<duration]

def norm_peak(coeff,duration):
    squared=np.array([0.])
    for c in coeff:squared=np.polyadd(squared,np.polymul(c,c))
    return math.sqrt(max(0.,max(np.polyval(squared,t) for t in extrema(squared,duration))))

class Trajectory:
    """Coordinate conversion only. Preserve the upstream trajectory clock and PVA."""
    def __init__(self,data,alignment,anchor=None):
        self.start=float(data['start']);self.ident=int(data['id'])
        self.duration=np.asarray(data['duration'],float)
        if (data['order']!=5 or not len(self.duration) or not np.isfinite(self.duration).all()
            or np.any(self.duration<=0) or not math.isfinite(self.start) or self.start<=0):
            raise ValueError('invalid trajectory format')
        self.c=np.stack([np.asarray(data[k],float).reshape(len(self.duration),6) for k in ('x','y','z')],axis=1)
        if not np.isfinite(self.c).all():raise ValueError('nonfinite coefficients')
        self.c=np.einsum('ab,nbc->nac',alignment.R,self.c);self.c[:,:,-1]+=alignment.t
        self.frame_R=alignment.R.copy();self.frame_t=alignment.t.copy()
        self.scale=1.;self.total=float(self.duration.sum())
    def translation_offset(self,lp,fp):
        return np.asarray(fp)-(self.frame_R@np.asarray(lp)+self.frame_t)
    def sample(self,now,position_offset=None):
        elapsed=now-self.start
        ended=elapsed>=self.total;t=min(self.total,max(0.,elapsed))
        i=min(len(self.duration)-1,int(np.searchsorted(np.cumsum(self.duration),t,side='right')))
        t-=float(self.duration[:i].sum())
        result=[np.array([np.polyval(np.polyder(axis,k),t) for axis in self.c[i]]) for k in range(3)]
        if ended:result[1][:]=0.;result[2][:]=0.
        if position_offset is not None:
            offset=np.asarray(position_offset,dtype=float)
            if offset.shape!=(3,) or not np.isfinite(offset).all():raise ValueError('invalid frame translation')
            result[0]+=offset
        return tuple(result)

class Session:
    """Pilot-owned mode selection. Planning decisions stay in the upstream FSM."""
    def __init__(self):
        self.phase='WAIT_FLIGHT';self.reason='';self.entered=False;self.hold=None
        self.entry_ros=None;self.entry_wall=None
        self.replan_since_ros=None;self.replan_min_start=None;self.replan_recoveries=0
        self.operator_stopped=False
        self.cycle_id=0
    def done(self,reason):self.phase='DONE';self.reason=reason;return None
    def manual_takeover(self,mode,stop=False):
        if self.operator_stopped or stop:return self.done('operator_stop_completed_in_manual_mode')
        self.entered=False;self.hold=None;self.entry_ros=None;self.entry_wall=None
        self.replan_since_ros=None;self.replan_min_start=None
        self.phase='MANUAL_WAIT';self.reason='pilot_manual_control:'+mode
        return None
    def step(self,now,rosnow,mode,armed,airborne,healthy,prepared,plan_ok,pose,reference,*,stop=False,**unused):
        if self.phase=='DONE':return None
        if self.entered and mode!='OFFBOARD':return self.manual_takeover(mode,stop=stop)
        if stop:
            if not self.entered:return self.done('operator_stopped_in_manual_mode' if self.cycle_id else 'operator_stopped_before_handover')
            if not self.operator_stopped:self.hold=tuple(pose) if pose else self.hold
            self.operator_stopped=True
        if not healthy or not armed or not airborne or not prepared:
            self.phase='WAIT_INPUT' if self.entered else 'WAIT_FLIGHT';self.reason='waiting_for_valid_inputs'
            return None
        if not self.entered and mode not in ('POSCTL','OFFBOARD'):
            self.phase='WAIT_FLIGHT';return None
        if not self.entered and mode=='POSCTL':
            self.phase='MANUAL_WAIT' if self.cycle_id else 'READY';self.reason=''
            return tuple(pose)
        if not self.entered:
            self.entered=True;self.entry_ros=rosnow;self.entry_wall=now
            self.cycle_id+=1
        if self.operator_stopped:
            self.phase='STOP_HOLD';self.reason='operator_stop_switch_back_to_POSCTL';return self.hold
        if plan_ok:
            if self.phase=='WAIT_PLAN':self.replan_recoveries+=1
            self.phase='TRACK';self.reason='';self.hold=None;return reference
        if self.hold is None:self.hold=tuple(pose)
        self.phase='WAIT_PLAN';self.reason='waiting_for_upstream_trajectory';return self.hold
