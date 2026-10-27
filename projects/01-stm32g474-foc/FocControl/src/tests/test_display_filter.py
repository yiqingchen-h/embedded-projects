"""显示滤波的信号质量、真实时间与缺口边界回归；不访问设备。"""

from __future__ import annotations

import numpy as np
import pytest
from foc_control.wave_plot import smooth_current


@pytest.mark.parametrize("rate", [5000, 10000])
def test_standard_reduces_noise_without_shifting_70hz(rate: int) -> None:
    """@brief 兼容档和高速档均抑制高频噪声，保留70Hz的幅值与相位。@param rate 点率。"""
    x = np.arange(rate) / rate
    signal = 0.020 * np.sin(2 * np.pi * 70 * x)
    noise = 0.004 * np.sin(2 * np.pi * 1500 * x) + 0.002 * np.cos(2 * np.pi * 2200 * x)
    original = signal + noise
    saved = original.copy()
    filtered = smooth_current(x, original, 2)
    core = slice(40, -40)
    basis = np.column_stack((np.sin(2 * np.pi * 70 * x), np.cos(2 * np.pi * 70 * x)))
    coefficients = np.linalg.lstsq(basis[core], filtered[core], rcond=None)[0]
    assert 0.97 < np.linalg.norm(coefficients) / 0.020 < 1.01
    assert abs(np.arctan2(coefficients[1], coefficients[0])) < 0.002
    assert np.std(filtered[core] - signal[core]) < 0.2 * np.std(noise[core])
    np.testing.assert_array_equal(original, saved)


def test_gap_and_nan_never_blend_neighboring_runs() -> None:
    """@brief 已知丢样和非有限样点隔开不同电平，不制造过渡或填补NaN。"""
    x = np.arange(301) / 10000
    y = np.r_[np.full(100, -0.020), np.full(100, 0.040), np.nan, np.full(100, 0.010)]
    gaps = np.zeros(len(x), dtype=bool)
    gaps[100] = True
    actual = smooth_current(x, y, 4, gaps)
    np.testing.assert_allclose(actual, y, atol=1e-14, equal_nan=True)


@pytest.mark.parametrize("backwards", [False, True])
def test_timestamp_discontinuity_resets_filter(backwards: bool) -> None:
    """@brief 时标长缺口和运行轮次回零均不借用前段数值。@param backwards 是否回零。"""
    x = np.r_[np.arange(100) / 10000, np.arange(100) / 10000 + (0 if backwards else 1)]
    y = np.r_[np.full(100, -0.020), np.full(100, 0.040)]
    np.testing.assert_allclose(smooth_current(x, y, 4), y, atol=1e-14)


@pytest.mark.parametrize("window", [0, 1, 2, 4])
def test_dc_and_short_fragments_stay_finite(window: int) -> None:
    """@brief 常值不偏置，片段不足时不造点、不产生非有限值。@param window 毫秒窗口。"""
    for count in (0, 1, 2, 3, 100):
        x = np.arange(count) / 10000
        y = np.full(count, 0.0123)
        actual = smooth_current(x, y, window)
        np.testing.assert_allclose(actual, y, atol=1e-14)
        assert len(actual) == count


def test_disabled_is_exact_and_strengths_are_monotonic() -> None:
    """@brief 原始档完全一致，增强平滑不制造过冲且逐步抑制高频分量。"""
    x = np.arange(2000) / 10000
    y = 0.020 * np.sin(2 * np.pi * 1000 * x)
    np.testing.assert_array_equal(smooth_current(x, y, 0), y)
    deviations = [float(np.std(smooth_current(x, y, width)[50:-50])) for width in (1, 2, 4)]
    assert deviations[0] > deviations[1] > deviations[2]
    assert np.max(np.abs(smooth_current(x, y, 4))) <= np.max(np.abs(y))


def test_missing_values_and_duplicate_times_stay_raw_when_no_interval() -> None:
    """@brief 缺少有效时差时保留原值，避免推测采样率。"""
    y = np.array([np.nan, 0.020, np.inf, -0.010])
    np.testing.assert_array_equal(smooth_current(np.zeros(4), y, 2), y)
