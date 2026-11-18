import numpy as np
import pytest
from ares_prerobot_test.profiles import Profile
from ares_prerobot_test.analysis import differentiate

@pytest.mark.parametrize('speed',[.1,3.])
def test_scurve_respects_bounds(speed):
    p=Profile('jerk',speed,1.,2.,1.)
    t=np.linspace(.001,p.ramp-.001,10000)
    v=np.array([p.up(x) for x in t]); dt=t[1]-t[0]
    a=np.gradient(v,dt); j=np.gradient(a,dt)
    assert max(a)<=1.+1e-5
    assert max(abs(j[3:-3]))<=2.+1e-4
    assert p.up(p.ramp)==pytest.approx(speed)
    assert p.sample(p.duration)[0]==0

def test_polynomial_derivatives_on_irregular_samples():
    rng=np.random.default_rng(42); t=np.cumsum(rng.uniform(.009,.011,1000))
    xy=np.column_stack([t**3/6,np.zeros_like(t)])
    grid,v,a,j,_=differentiate(t,xy)
    assert np.max(abs(v[:,0]-grid**2/2))<.01
    assert np.max(abs(a[:,0]-grid))<.03
    assert np.median(j[:,0])==pytest.approx(1.,abs=.02)

def test_missing_feedback_is_not_interpolated_as_motion():
    t=np.r_[np.arange(0,1,.01),np.arange(2,3,.01)]
    with pytest.raises(ValueError,match='gap'):
        differentiate(t,np.column_stack([t,t]))

def test_out_of_order_stamps_rejected():
    t=np.arange(100)*.01; t[30]=t[29]
    with pytest.raises(ValueError,match='increase'):
        differentiate(t,np.column_stack([t,t]))

def test_imu_gravity_removed_with_tilted_sensor():
    from scipy.spatial.transform import Rotation
    from ares_prerobot_test.analysis import imu_metrics
    rotation=Rotation.from_euler('xyz',[.2,-.3,.5])
    raw=rotation.inv().apply([0.,0.,9.80665]); q=rotation.as_quat()
    rows=[]
    for t in np.arange(0.,3.,.01):
        rows.append(dict(stamp=t,orientation_available=1,frame='imu_link',
                         **dict(zip(('ax','ay','az'),raw)),**dict(zip(('qx','qy','qz','qw'),q))))
    result=imu_metrics(rows,9.80665,'imu_link')
    assert result['available']
    assert result['peak_horizontal_acceleration']<1e-10
    assert result['peak_horizontal_jerk']<1e-10


def test_imu_without_orientation_is_not_reported_as_ground_acceleration():
    from ares_prerobot_test.analysis import imu_metrics
    result=imu_metrics([dict(orientation_available=0,frame='imu_link')]*20,9.80665,'imu_link')
    assert not result['available']
