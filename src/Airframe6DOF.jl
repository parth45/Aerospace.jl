############################################################################
#
#  Airframe.jl
#
#  Purpose:    Single file that contains all functions needed to simulate a
#              6-DOF Airframe.
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################

# Initialize Position
function InitPos( time, latitude, longitude, altitude )

    ## Transformation Matrices ECEF, ECI, NED
    Tr_ei = TR_EI(time);
    Tr_ie = Tr_ei';
    Tr_ne = TR_NE(latitude, longitude);

    ## Position Vectors
    rm_ecef = LLH2ECEF(latitude, longitude, altitude);
    rm_eci = Tr_ie*rm_ecef;

    ## Populating Position Structure
    position = WGS_Pos(time, latitude, longitude, altitude, rm_ecef, rm_eci, Tr_ei, Tr_ne);
    return position;
end

############################################################################
# Initialize Velocity
function InitVel( vmag::Float64, gamma::Float64, azimuth::Float64, position::WGS_Pos )

    time = position.time;
    Tr_ei = position.Tr_ei;
    Tr_ie = Tr_ei';
    Tr_ne = position.Tr_ne;
    Tr_en = Tr_ne';

    wb_ei_eci = [0.0;0.0;WEII3];

    vm_ned = [
        vmag*cos(gamma)*cos(azimuth)
        vmag*cos(gamma)*sin(azimuth)
        -vmag*sin(gamma)
    ];

    vm_ecef = Tr_en*vm_ned;
    vm_eci = Tr_ie*vm_ecef + cross(wb_ei_eci, position.rm_eci);

    velocity = Velocity_States( time, vmag, gamma, azimuth, vm_ned, vm_ecef, vm_eci );
    return velocity;
end

############################################################################
# Initialize Euler
function InitEuler( Roll::Float64, Pitch::Float64, Heading::Float64 )
    Tr_bn = TR_BN( Roll, Pitch, Heading);

    euler = Euler_Angles(Roll, Pitch, Heading, Tr_bn);

    return euler;
end

############################################################################
# Initialize Rates
function InitAngleRates(wb_bn_body::Array{Float64,1}, position::WGS_Pos, euler::Euler_Angles)

    ## Transformation Matrices
    Tr_ei = position.Tr_ei;
    Tr_ie = Tr_ei';
    Tr_ne = position.Tr_ne;
    Tr_en = Tr_ne';
    Tr_bn = euler.Tr_bn;
    Tr_nb = Tr_bn';
    Tr_bi = Tr_bn*Tr_ne*Tr_ei;
    Tr_ib = Tr_bi';

    ## Earth Rotation Rate
    wb_ne_body = [0.0;0.0;0.0];
    wb_ne_eci = Tr_ib*wb_ne_body;
    wb_ei_eci = [0.0;0.0;WEII3];
    wb_ei_ecef = Tr_ei*wb_ei_eci;
    wb_bi_body = wb_bn_body + wb_ne_body + Tr_bi*wb_ei_eci;

    ## Transport Rate
    angle_rates = Angle_Rates( wb_bi_body, wb_bn_body, wb_ei_eci, wb_ei_ecef, wb_ne_eci )
    return angle_rates
end

############################################################################
# Initialize
function Airframe_Initialize( tsim::Float64,
                     time::Float64, dt::Float64,
                     position::WGS_Pos, velocity::Velocity_States,
                     euler::Euler_Angles, angle_rates::Angle_Rates, mass_props::MassProps)

    ## Time Computations
    dt_eci = time - tsim;

    ## Transformation Matrices
    Tr_ei = position.Tr_ei;
    Tr_ie = Tr_ei';
    Tr_ne = position.Tr_ne;
    Tr_en = Tr_ne';
    Tr_bn = euler.Tr_bn;
    Tr_nb = Tr_bn';
    Tr_bi = Tr_bn*Tr_ne*Tr_ei;
    Tr_ib = Tr_bi';

    quat = QuatInit( Tr_bi );
    quat_bi = Quaternions(quat[1], quat[2], quat[3], quat[4] );

    xo = [ position.rm_eci; velocity.vm_eci; angle_rates.wb_bi_body; quat];

    ## Quad Rotor Intialization
    airframe_init = Aero6DOF(time, dt, position, velocity, euler, angle_rates, mass_props, quat_bi, xo);

    return airframe_init
end

############################################################################
# Update
function Airframe_Update(airframe::Aero6DOF,
                ftot_body::Array{Float64,1},
                mtot_body::Array{Float64,1},
                )
    dt = airframe.dt;
    airframe_out = airframe;

    ## Time
    time = airframe.time
    wb_ei_eci = [0.0;0.0;WEII3];
    wb_ei_ecef = wb_ei_eci;

    ## Transformation Matrices
    Tr_ei = airframe.position.Tr_ei;
    Tr_ie = Tr_ei';
    Tr_ne = airframe.position.Tr_ne;
    Tr_en = Tr_ne';
    Tr_bn = airframe.euler.Tr_bn;
    Tr_nb = Tr_bn';
    Tr_bi = Tr_bn*Tr_ne*Tr_ei;
    Tr_ib = Tr_bi';

    ## Mass Properties
    mass = airframe.massprops.mass;
    II = airframe.massprops.II;
    cg_vec_body = airframe.massprops.cg;

    ## Quadrotor Velocity
    vel = airframe.velocity;
    vm_eci = airframe.velocity.vm_eci;
    vm_ecef = Tr_ei*vm_eci - cross(wb_ei_ecef, airframe.position.rm_ecef);
    vm_ned = Tr_ne*vm_ecef;
    vm_body = Tr_bn*vm_ned;

    ## Quadrotor Position
    pos = airframe.position;

    ## Attitude
    wb_body = airframe.angle_rates;
    wb_bi_body = wb_body.wb_bi_body;

    ## Quaternions
    quat_bi = airframe.quat_bi;
    quat_bi_vec = [quat_bi.q0; quat_bi.q1; quat_bi.q2; quat_bi.q3];

    ## State Vector
    xo = [ airframe.position.rm_eci;
           airframe.velocity.vm_eci;
           airframe.angle_rates.wb_bi_body;
           quat_bi_vec;
           ];


    ## Gravity
    grav_ecef = WGS84_GRAVITY(airframe.position.rm_ecef);
    grav_eci = Tr_ie*grav_ecef - cross( wb_ei_eci, cross(wb_ei_eci, airframe.position.rm_eci) );
    grav_ecef = Tr_ei*grav_eci;
    grav_body = Tr_bi*grav_eci;
    grav_ned = Tr_nb*grav_body;
    fgrav_body = mass*grav_body;

    ## Total Forces in Body Frame
    acc_meas_body = ftot_body/mass;
    ftot_eci = Tr_ib*ftot_body;

    ## Computing Derivatives
    dx_pos = vm_eci;
    dx_vel = Tr_ib*(1.0/mass*( ftot_body + fgrav_body - cross( wb_bi_body, mass*vm_body ) ) );
    dx_wb  = II^-1*( mtot_body - cross( wb_bi_body, II*wb_bi_body ) );
    dx_quat = QuatDerivative(quat_bi_vec, wb_bi_body)

    ## Derivative Vector
    dx = [dx_pos; dx_vel; dx_wb; dx_quat];

    ## Euler Integration
    xo = xo + dt*dx;
    time = time + dt;

    ## Updating Parameters
    airframe_out.xo = xo;
    airframe_out.time = time;

    ## Updating States
    rm_eci = xo[1:3];
    vm_eci = xo[4:6];
    wb_bi_body = xo[7:9];
    quat_bi_body = xo[10:13];

    ## Updating Quaternions
    quat_bi_body = quat_bi_body/norm(quat_bi_body);
    airframe_out.quat_bi = Quaternions(quat_bi_body[1],
                                   quat_bi_body[2],
                                   quat_bi_body[3],
                                   quat_bi_body[4] );

    ## Updating Transformation Matrix ECI to ECEF;
    Tr_ei = TR_EI(time);
    Tr_ie = Tr_ei';

    ## Updating Positon
    rm_ecef = Tr_ei*rm_eci;
    LLH = ECEF2LLH(rm_ecef);
    latitude = LLH[1];
    longitude = LLH[2];
    altitude = LLH[3];
    pos = InitPos( time, latitude, longitude, altitude );
    airframe_out.position = pos;

    ## Updating Transformations
    Tr_bi = QuatToDCM(quat_bi_body);
    Tr_ib = Tr_bi';
    Tr_ne = pos.Tr_ne;
    Tr_en = Tr_ne';
    Tr_bn = Tr_bi*Tr_ie*Tr_en;
    Tr_nb = Tr_bn';
    Tr_eb = Tr_en*Tr_nb;
    Tr_be = Tr_eb';

    ## Updating Euler
    eulervec = ComputeEuler( Tr_bn );
    roll = eulervec[1];
    pitch = eulervec[2];
    heading = eulervec[3];
    airframe_out.euler = InitEuler( roll, pitch, heading );

    ## Updating Body Rates
    wb_bi_ecef = Tr_eb*wb_bi_body;
    wb_ei_ecef = wb_ei_eci;
    wb_ei_body = Tr_be*wb_ei_ecef;
    wb_bn_body = wb_bi_body - wb_ei_body;
    airframe_out.angle_rates = InitAngleRates(wb_bn_body, airframe_out.position, airframe_out.euler);

    ## Updating Velocity
    vm_ecef = Tr_ei*vm_eci - cross(wb_ei_ecef, pos.rm_ecef);
    vm_ned = pos.Tr_ne*vm_ecef;
    vmag = norm(vm_ned);
    gamma = atan2( -vm_ned[3], sqrt(vm_ned[1]^2 + vm_ned[2]^2));
    azimuth = atan2( vm_ned[2], vm_ned[1] );
    vel = InitVel( vmag, gamma, azimuth, pos );
    airframe_out.velocity = vel;

    return airframe_out;
end
