import numpy
import matplotlib.pyplot as plt

def parallelhybrid(x):
    #####################################################################
    # This files calculates torque split for simulink model.
    # Given inputs from the driving cycle and controller this script should return The torques on the electric motor and combustion engine.
    #####################################################################

    # Input data from Simulink Model
    w_ice = x[0] # Angular Speed for ICE
    dw_ice = x[1] # Angular Acceleration for ICE
    T_req = x[2] # Torque Request or demand
    lambdaa = x[3] # Equivalence Factor

    # Vehicle Parameters:-
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


    #-----------------------------------Torque Split Calculations--------------------------------#

    # Battery Model
    I = numpy.arange(Imin, Imax, 0.0001)

    Powerbattery = (Uoc * I) - (Ri * I ** 2)

    # Electric Machine Model
    Power_electric_machine = Powerbattery * n_electricmachine ** numpy.sign(I) # Power of Electric Machine

    T_em = (Powerbattery * n_electricmachine **  numpy.sign(I)) / w_ice # Torque of Electric Machine

    # Engine Model
    T_ice = T_req - T_em # Torque of Engine

    x = ((p_me0 * V_disp) / (4 * 3.14)) # First Part to calculate Fuel consumption

    y = Je * dw_ice # Second Part to calculate Fuel consumption

    costVector = (w_ice / (e * H_l)) * (T_ice + x + y) # Fuel Consumption Cost Vector for given t_vec

    # Hamiltonian Calculation [Torque Split]
    P_f = H_l * costVector # Fuel Power
    P_f[costVector < 0] = 0 # Constraints
    P_ech = I * Uoc # Electro - chemical Power for Battery

    if w_ice==0:
        T_ice = 0
        T_em = 0
        u = [T_ice, T_em]
    else:
        H = P_f + (lambdaa * P_ech)
        H[(T_ice + (Je * dw_ice)) > T_engine_max] = numpy.inf
        H[numpy.logical_or(T_em > T_em_max, T_em < -T_em_max)  ] = numpy.inf
        H[numpy.logical_or(Power_electric_machine < -P_em_max * 1000, Power_electric_machine > P_em_max * 1000)  ] = numpy.inf
        i = numpy.argmin(H)
        T_ice = T_ice[i]
        T_em = T_em[i]
        u = [T_ice, T_em]

    u = numpy.array(u)
    return T_ice, T_em


