"""
ThermalCalculations.py — Resin + mold lumped thermal model (2-node energy balance).

Can be used standalone or imported by DispenseUI.py to run simulations
with STL-derived geometry values.

Modeling choices:
- Resin is a single lumped node (uniform temperature Tr)
- Mold is a single lumped node (uniform temperature Tm)
- Heat flows resin -> mold via wall conduction
- Mold loses heat to ambient via convection + radiation
- Resin generates heat during cure with a simple power profile

All computations are done in SI units (m, kg, s, W, J, K).
"""

import numpy as np

# ── Material constants (we can edit as needed) ──────────────────────────────────────
T_AMB_K = 295.15  # ambient temperature (K)
K_MOLD = 0.2  # mold thermal conductivity W/(m*K)
EPS_MOLD = 0.9 # mold emissivity (-)
SIGMA = 5.67e-8 # Stefan-Boltzmann constant W/(m^2*K^4)
H_CONV = 13.6  # convection coefficient W/(m^2*K)
CP_MOLD = 1400.0  # mold specific heat J/(kg*K)
CP_RESIN = 1800.0  # resin specific heat J/(kg*K)
HT_RESIN = 334.0 # total exotherm J/g
RESIN_DENSITY = 1.2 # g/mL
MOLD_DENSITY = 1.24   # g/cm^3  (PLA ~1.24)
T_REACT_S = 360.0 # reaction time (s)
T_END_S = 2000.0  # total simulation time (s)
DT = 0.2 # time step (s)


def _q_gen_constant(t, Q_total_J, t_react_s):
    """Constant power during [0, t_react_s], then zero."""
    return (Q_total_J / t_react_s) if (0.0 <= t <= t_react_s) else 0.0


def simulate(
    m_resin_g: float,
    A_surface_cm2: float,
    m_mold_g: float,
    wall_thickness_mm: float = 1.0,
    dt: float = DT,
    t_end_s: float = T_END_S,
):
    """
    Run the 2-node thermal simulation.

    Parameters
    ----------
    m_resin_g : float       Mass of resin in grams (cavity_ml * resin_density).
    A_surface_cm2 : float   Mold outer surface area in cm^2.
    m_mold_g : float        Mass of mold in grams (solid_ml * mold_density).
    wall_thickness_mm : float  Average wall thickness in mm.
    dt : float              Time step (s).
    t_end_s : float         Total simulation time (s).

    Returns
    -------
    t : ndarray             Time array (s).
    Tr : ndarray            Resin temperature (K).
    Tm : ndarray            Mold temperature (K).
    peak_resin_C : float    Peak resin temperature (deg C).
    peak_mold_C : float     Peak mold temperature (deg C).
    """
    # Unit conversions
    A_perp = A_surface_cm2 * 1e-4          # m^2 (conduction area ~ surface area)
    A_tot = A_surface_cm2 * 1e-4           # m^2 (outer surface for losses)
    m_mold = m_mold_g / 1000.0             # kg
    m_resin = m_resin_g / 1000.0           # kg
    L = wall_thickness_mm * 1e-3           # m

    C_mold = m_mold * CP_MOLD
    C_resin = m_resin * CP_RESIN
    Q_total_J = HT_RESIN * m_resin_g
    R_wall = L / (K_MOLD * A_perp)         # K/W

    n = int(t_end_s / dt) + 1
    t = np.linspace(0.0, t_end_s, n)
    Tr = np.empty(n)
    Tm = np.empty(n)
    Tr[0] = T_AMB_K
    Tm[0] = T_AMB_K

    for i in range(n - 1):
        ti = t[i]
        q_wall = (Tr[i] - Tm[i]) / R_wall
        q_conv = H_CONV * A_tot * (Tm[i] - T_AMB_K)
        q_rad = EPS_MOLD * SIGMA * A_tot * (Tm[i]**4 - T_AMB_K**4)
        q_out = q_conv + q_rad
        q_rxn = _q_gen_constant(ti, Q_total_J, T_REACT_S)

        dTr_dt = (q_rxn - q_wall) / C_resin
        dTm_dt = (q_wall - q_out) / C_mold

        Tr[i + 1] = Tr[i] + dTr_dt * dt
        Tm[i + 1] = Tm[i] + dTm_dt * dt

    peak_resin_C = float(Tr.max() - 273.15)
    peak_mold_C = float(Tm.max() - 273.15)

    return t, Tr, Tm, peak_resin_C, peak_mold_C


# Standalone mode with example parameters
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Example defaults for standalone testing
    m_resin_g = 12.0
    A_surface_cm2 = 25.0
    m_mold_g = 13.0

    t, Tr, Tm, peak_r, peak_m = simulate(m_resin_g, A_surface_cm2, m_mold_g)

    print(f"Peak resin temperature: {peak_r:.2f} C")
    print(f"Peak mold  temperature: {peak_m:.2f} C")

    plt.figure()
    plt.plot(t, Tr - 273.15, label="Resin Tr (C)")
    plt.plot(t, Tm - 273.15, label="Mold  Tm (C)")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (C)")
    plt.title("Temperature vs Time")
    plt.legend()
    plt.show()
