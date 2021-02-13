# ECMS_HEV_Python_SIMULINK

Real-time Optimal Control of Hybrid Electric Powertrains
Equivalent consumption minimization strategy (ECMS) Optimization of Hybrid Vehicle Fuel Consumption in Python. 
Run the file 'HEV_ECMS.mdl'.

# Files for the optimal control of parallel hybrid configurations using Equivalent consumption minimization strategy (ECMS)
 ================================================================

# Vehicle Model
 =====================
 HEV_ECMS.mdl      - A basic vehicle model

# Problem specific scrips and functions
 =====================================
 init_HEV_ECMS.m    - Initialize data
 parallelhybrid_ECMS.py - Torque Split function file

(C) 2020 Ruchit Bhikadiya
 
1. Main Assignment information:
A hybrid vehicle has various possibilities of configurations. The parallel vehicle is the 
well known concepts. The parallel hybrid has a mechanical link (via transmission) from the combustion 
engine to the wheels. This means that for the parallel hybrid only the state of charge is optimized.
The models for the parallel, should be formulated so that the cost for following an arc in 
the optimal control problem can be calculated.

2. Vehicle Parameters:
    # Lower Heating Value
    H_l = 44.6e6  # J/kg

    # Fuel Density
    roh_l = 732.2  # Kg/m3

    # Air Density
    roh_a = 1.18  # kg/m3

    # Engine Inertia
    Je = 0.2  # kgm2

    # Engine Maximum Torque
    T_engine_max = 115  # Nm

    # Engine Displacment
    V_disp = 1.497e-3  # m3

    # Willans approximation of engine efficiency and pressure
    e = 0.4
    p_me0 = 0.1e6  # MPa

    # Battery charging capacity
    Q_o = 6.5  # Ah

    # Open circuit voltage
    Uoc = 300  # V

    # Maximum dis-/charging current
    Imax = 200  # A
    Imin = -200  # A

    # Inner resistance
    Ri = 0.65  # ohm

    # Efficiency of electrical machine
    n_electricmachine = 0.9

    # Gravity
    g = 9.81

    # Drag coefficient
    cD = 0.32

    # Rolling resistance coefficient
    cR = 0.015

    # Frontal area
    Af = 2.31  # m2

    # Vehicle mass
    mv = 1500  # kg

    # Wheel radius
    rw = 0.3  # m

    # Inertia of the wheels
    Jw = 0.6  # kgm2
    mwheel = Jw / (rw ** 2)

    # Efficiency of Transmission
    eta_gearbox = 0.98

    # Electric machine Maximum Torque
    T_em_max = 400  # Nm

    # Power of Electric Machine
    P_em_max = 50  # kW

    # Electric motor weight
    m_em = 1.5  # kg/Kw

    # Maximum powertrain power
    P_pt_max = 90.8  # kW
