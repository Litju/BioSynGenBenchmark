"""Generator-independent exact rational CI references for manufactured fixtures."""
from fractions import Fraction as F

G = F(196133, 20000)
EXPECTED = {
    "support_left_n": (F(37,10)*G/2 + F(21,10)/(2*F(21,50)), F(1,10**10)),
    "support_right_n": (F(37,10)*G/2 - F(21,10)/(2*F(21,50)), F(1,10**10)),
    "capture_velocity_m_s": (F(0), F(1,10**12)),
    "capture_normal_impulse_n_s": (F(23,10)*G*F(1,50), F(1,10**12)),
    "capture_tangent_impulse_n_s": (-F(23,10)*F(1,100), F(1,10**12)),
    "impact_velocity_m_s": (F(3,10)*F(17,10), F(1,10**12)),
    "impact_impulse_n_s": (F(16,5)*F(13,10)*F(17,10), F(1,10**12)),
    "impact_work_j": (-F(16,5)*(1-F(3,10)**2)*F(17,10)**2/2, F(1,10**12)),
}


def rational_record(value):
    return {"numerator": value.numerator, "denominator": value.denominator}
