import math

from ares_prerobot_test.motion import Motion


def test_planar_and_angular_limits_through_target_changes():
    motion = Motion(angular_accel=0.8, angular_jerk=3.0)
    dt = 0.01
    previous_a = motion.a[:]
    for target in ((1.0, 0.5, 0.6), (-0.5, -0.6, -0.4), (0.0, 0.0, 0.0)):
        motion.set_target(*target, amax=1.2, jerkmax=4.0)
        for _ in range(600):
            motion.step(dt)
            assert math.hypot(*motion.a[:2]) <= 1.2 + 1e-9
            assert math.hypot(motion.a[0]-previous_a[0], motion.a[1]-previous_a[1]) <= 4.0*dt + 1e-9
            assert abs(motion.a[2]) <= 0.8 + 1e-9
            assert abs(motion.a[2]-previous_a[2]) <= 3.0*dt + 1e-9
            previous_a = motion.a[:]
    assert motion.at_target()


def test_explicit_halt_resets_command_state():
    motion = Motion()
    motion.set_target(0.5, 0.0, 0.2, 1.0, 2.0)
    for _ in range(50):
        motion.step(0.01)
    motion.halt()
    assert motion.at_target()
