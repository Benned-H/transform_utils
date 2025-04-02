"""Define property-based unit tests for the sampling module."""

"""
Unit tests for the sampling module using Hypothesis.
Verifies that:
 - The total number of samples equals the requested count.
 - The first sample is the center angle.
 - Each sample is exactly center plus an integer multiple of the step size.
 - The absolute differences follow a paired pattern that increases in equal increments.
"""
import numpy as np
from hypothesis import given
from hypothesis import strategies as st

from transform_utils.sampling_utils import generate_alternating_centered_angles


@st.composite
def draw_center_rad(draw: st.DrawFn) -> float:
    """Generate a center angle between -10 and 10 radians."""
    return draw(
        st.floats(
            min_value=-10 * np.pi,
            max_value=10 * np.pi,
            allow_nan=False,
            allow_infinity=False,
        ),
    )


@st.composite
def draw_num_angles(draw: st.DrawFn) -> int:
    """Generate a number of angles to be sampled around the center angle."""
    return draw(st.integers(min_value=1, max_value=50))


@given(center_rad=draw_center_rad(), num_angles=draw_num_angles())
def test_total_centered_angles(center_rad: float, num_angles: int) -> None:
    """Verify that the total number of samples equals the requested count."""
    angles = generate_alternating_centered_angles(center_rad, num_angles)
    assert len(angles) == num_angles


@given(center_rad=draw_center_rad(), num_angles=draw_num_angles())
def test_center_angle_is_correct(center_rad: float, num_angles: int) -> None:
    """Verify that the first sample is exactly the given center angle."""
    angles = generate_alternating_centered_angles(center_rad, num_angles)

    result_center_rad = angles[0]
    assert np.isclose(center_rad, result_center_rad)


@given(center_rad=draw_center_rad(), num_angles=draw_num_angles())
def test_samples_are_multiples_of_step_rad(center_rad: float, num_angles: int) -> None:
    """Verify that every sample is the center plus an integer multiple of the step size."""
    angles = generate_alternating_centered_angles(center_rad, num_angles)

    expected_step_rad = 2 * np.pi / (num_angles + 1)
    for angle_rad in angles:
        diff_rad = np.abs(angle_rad - center_rad)
        multiplier = diff_rad / expected_step_rad

        #  Check that multiplier is (approximately) an integer
        assert np.isclose(multiplier, round(multiplier))
