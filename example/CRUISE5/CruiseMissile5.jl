############################################################################
#
#  CruiseMissile5.jl
#
#  Purpose:    Top level quadrotor file
#
#  Author:     Edward Daughtery
#
#
#  Date:       05 August 2016
#
#
############################################################################
# Mass Properties
#    This function provides the mass properties for a simple quadrotor
function Mass_Props()
    mass = 1129.96; # Kg

    return mass;
end

############################################################################
# Initialize Quadrotor Structure
function MissileInit( time::Float64, dt::Float64, position_init::WGS_Pos, vel_init::Velocity_States, euler_init::Euler_Angles, rate_init::Angle_Rates )

    # Mass Properties
    mass = Mass_Props();

    # Airframe Initialization
    missile_airframe = Airframe5_Initialize( time, time, dt, position, velocity, euler, angle_rates, mass );

    # Navigation Message Intialization
    zvec = [0.0;0.0;0.0];
    nav_state = NavStates( time, position, velocity, rate_init, zvec, zvec, euler, missile_airframe.quat_bi );

    ## Software Initialization
    swdt = 0.01;   # Software runs at 100 Hz
    swMsg = Initialize_SW(time, swdt, nav_state);

    ## Controls
    zz = 0.0;
    control = Controls(zz,zz,zz,zz,zz);

    # Creating Vehicle Structure
    missile = Missile( time, dt, missile_airframe, control, swMsg);

    return missile;
end


############################################################################
# Missile Update
function MissileUpdate(missile::Missile)

    vehout = missile;

    ## Taking Measurements
    time = missile.time;
    dt = missile.dt;
    position = missile.airframe.position;
    alt = position.altitude;
    velocity = missile.airframe.velocity;
    rate = missile.airframe.angle_rates;
    rate_meas = missile.airframe.angle_rates.wb_bi_body;
    accel_meas = missile.airframe.accel_meas;
    euler = missile.airframe.euler;
    quat_bi = missile.airframe.quat_bi;
    nav_state = NavStates( time, position, velocity, rate, rate_meas, accel_meas, euler, quat_bi );

    # Software Update
    swMsg = Update_SW( missile.software);
    swMsg.navstates = nav_state;
    missile.software = swMsg;

    ## Control Variables
    alpha = swMsg.controls.alpha;
    alphadot = swMsg.controls.alphadot;
    phi = swMsg.controls.phi;
    phidot = swMsg.controls.phidot;
    throttle = swMsg.controls.throttle;

    # Wind Relative Vector
    wind_ned = zeros(3);
    vm_windrel_ned = wind_ned - velocity.vm_ned;
    true_airspeed = norm(vm_windrel_ned);

    # Atomosphere
    atmdata = Atmosphere(alt, true_airspeed);
    Press = atmdata[1];
    Temp = atmdata[2];
    rho = atmdata[3];
    aa = atmdata[4];
    Mach = atmdata[5];
    qbar = atmdata[6];

    # Aerodynamics
    Tr_bv = TR_BV(alpha,phi)
    faero_body = Aerodynamics( Mach, alpha, phi, qbar, alt, Tr_bv);

    # Propulsion
    fprop_body = Propulsion( throttle );

    # Total Forces
    ftot_body = faero_body + fprop_body;

    # Airframe Update
    vehout.airframe = Airframe5_Update(vehout.airframe, ftot_body, alpha, alphadot, phi, phidot)
    vehout.time = vehout.airframe.time;
    vehout.control = swMsg.controls;
    vehout.software = swMsg;

    return vehout;
end

############################################################################
# Missile Aerodynamics
function Aerodynamics( Mach::Float64, Alpha::Float64, roll::Float64, qbar::Float64, alt::Float64, Tr_bv::Array{Float64,2})

    alpha = Alpha*r2d;
    alpha = Bound(alpha,-6.0, 9.0);
    Sref = 1.2;

    ## Table Lookup on Coeficient of Lift
    #
    cl = 0.105229367851008 + 0.110086637937226*alpha + 0.848775782653339*Mach + -0.00219199346405218*alpha*alpha +
        -1.02349751408577*Mach*Mach + 0.00882582776700423*alpha*Mach;
    cl = cl;

    cd = (0.167813288903029 + -0.0016035101720176*alpha + -0.501792617254165*Mach + 0.00145285655929036*alpha*alpha +
        0.47032808316194*Mach*Mach + -0.00159279968397613*alpha*Mach);

    faero_vel = [-cd;0.0;-cl]

    ## Computing Coefficients in Body Frame
    faero_body = Tr_bv*faero_vel;
    cx = faero_body[1];
    cy = faero_body[2];
    cz = faero_body[3];

    ## Forces in Body Frame
    forces_body = zeros(3);
    forces_body[1] = qbar*Sref*cx;
    forces_body[2] = qbar*Sref*cy;
    forces_body[3] = qbar*Sref*cz;

    return forces_body;
end

############################################################################
# Missile Propulsion
function Propulsion( throttle::Float64)

    lb2Nt = 4.44822162;

    Throttle = Bound(throttle, 0.0, 100.0);
    thrust = 150.0*Throttle*lb2Nt;

    fb_prop_body = zeros(3);
    fb_prop_body[1] = thrust;

    return fb_prop_body;
end
