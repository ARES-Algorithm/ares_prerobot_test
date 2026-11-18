import math

from ares_prerobot_test.telemetry import LiveTelemetry


def test_wheel_current_pairing_and_derivatives():
    clock = [100.]
    live = LiveTelemetry(clock=lambda: clock[0])
    for i in range(20):
        clock[0] = 100. + i * .02
        sec, usec = divmod(i * 20000, 1000000)
        # v(t) = t²; RPM is measured motor speed before conversion.
        v = (i * .02) ** 2
        rpm = v * 60 / (2 * math.pi * .04)
        live.command(v, 0, clock[0])
        live.raw(0x0131, [i, sec, usec, rpm, rpm, rpm, rpm, 0, 0, 0, 0])
        live.raw(0x0141, [i, sec, 1., 2., 3., 4., 4., 5., 6., 7.])
    data = live.snapshot(0)
    assert len(data['wheel']) == len(data['current']) == len(data['current_mean']) == 20
    assert data['current'][-1]['t'] == data['wheel'][-1]['t']
    assert data['current_mean'][-1]['t'] == data['wheel'][-1]['t']
    assert data['wheel'][-1]['derived']['j'] == pytest.approx(2., abs=.01)
    assert data['current'][-1]['Id'] == 1.
    assert data['current'][-1]['Is'] == 4.
    assert data['current_mean'][-1]['Id'] == 2.5
    assert data['current_mean'][-1]['Is'] == 5.5
    cursor = data['cursor']
    assert not live.snapshot(0, cursor)['current']
    assert not live.snapshot(0, cursor)['current_mean']


def test_gap_and_wrong_firmware_do_not_create_false_line():
    clock = [10.]
    live = LiveTelemetry(clock=lambda: clock[0])
    live.raw(0x0131, [0.] * 15)
    assert '格式不匹配' in live.snapshot()['error']
    live.raw(0x0131, [1, 1, 0, 60, 0, 0, 0, 0, 0, 0, 0])
    clock[0] = 11.
    live.raw(0x0131, [2, 2, 0, 60, 0, 0, 0, 0, 0, 0, 0])
    assert live.snapshot()['wheel'][-1]['derived'] is None
    live.raw(0x0131, [2, 2, 0, 60, 0, 0, 0, 0, 0, 0, 0])
    assert len(live.snapshot()['wheel']) == 2


def test_current_mean_requires_all_four_readings():
    live = LiveTelemetry(clock=lambda: 10.)
    live.raw(0x0131, [1, 1, 0, 60, 60, 60, 60, 0, 0, 0, 0])
    live.raw(0x0141, [1, 1, 1, 2, float('nan'), 4, 5, 6, 7, 8])
    mean = live.snapshot()['current_mean'][0]
    assert mean['Id'] is None
    assert mean['Is'] == 6.5


def test_current_arriving_before_rpm_survives_incremental_poll():
    live = LiveTelemetry(clock=lambda: 10.)
    live.raw(0x0141, [1, 1, 1, 2, 3, 4, 5, 6, 7, 8])
    cursor = live.snapshot()['cursor']
    assert live.snapshot()['current'] == []
    live.raw(0x0131, [1, 1, 0, 60, 60, 60, 60, 0, 0, 0, 0])
    data = live.snapshot(since=cursor)
    assert len(data['current']) == len(data['current_mean']) == 1
    assert data['current_mean'][0]['Id'] == 2.5
    assert live.snapshot(since=data['cursor'])['current'] == []


import pytest
