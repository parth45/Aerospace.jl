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
function Airframe5_Initialize( tsim::Float64,
                     time::Float64, dt::Float64,
                     position::WGS_Pos, velocity::Velocity_States,
                     euler::Euler_Angles, angle_rates::Angle_Rates, mass::Float64)

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

    zz = [0.0;0.0;0.0];

    xo = [ position.rm_eci; velocity.vm_eci];

    ## Quad Rotor Intialization
    airframe_init = Aero5DOF(time, dt, position, velocity, euler, angle_rates, zz,  mass, quat_bi, xo);

    return airframe_init
end

############################################################################
# Update
function Airframe5_Update( airframe::Aero5DOF, ftot_body::Array{Float64,1}
			   ,alpha::Float64, alphadot::Float64, phi::Float64, phidot::Float64)
    dt = airframe.dt;
    airframe_out = airframe;

    xo = airframe.xo;

    rm_eci = airframe.position.rm_eci;
    vm_eci = airframe.velocity.vm_eci;

    ## Time
    time = airframe.time;
    wb_ei_eci = [0.0;0.0;WEII3];
    wb_ei_ecef = wb_ei_eci;

    ## Transformation Matrices
    Tr_ei = airframe.position.Tr_ei;
    Tr_ie = Tr_ei';
    Tr_ne = airframe.position.Tr_ne;
    Tr_en = Tr_ne';
    Tr_bn = airframe.euler.Tr_bn;
    Tr_nb = Tr_bn';

    ## Mass Properties
    mass = airframe.mass;

    ## Airframe Velocity
    vel = airframe.velocity;
    vm_eci = airframe.velocity.vm_eci;
    vm_ecef = Tr_ei*vm_eci - cross(wb_ei_ecef, airframe.position.rm_ecef);
    vm_ned = Tr_ne*vm_ecef;
    vm_body = Tr_bn*vm_ned;
    vmag = vel.vmag;
    gamma = vel.gamma;
    azimuth = vel.azimuth;

    ## Finishing Computation of Transformation Matrices
    Tr_vn = TR_VN(gamma,azimuth);
    Tr_nv = Tr_vn';
    Tr_bv = TR_BV(alpha, phi);
    Tr_vb = Tr_bv';
    Tr_bi = Tr_bv*Tr_vn*Tr_ne*Tr_ei;
    Tr_ib = Tr_bi';

    ## Airframe Position
    pos = airframe.position;

    ## Quaternions
    quat_bi = airframe.quat_bi;
    quat_bi_vec = [quat_bi.q0; quat_bi.q1; quat_bi.q2; quat_bi.q3];

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
    airframe_out.accel_meas = acc_meas_body;

    ## Total Acceleration
    acc_tot_body = acc_meas_body + grav_body;
    acc_tot_ned = Tr_nb*acc_tot_body;
    acc_tot_vel = Tr_vn*acc_tot_ned;

    ## Gamma Dot and Azimuth Dot
    gamdot  = -acc_tot_vel[3]/vmag;
    azdot   = acc_tot_vel[2]/(vmag*cos(gamma));

    ## Rotation Rate of Velocity Frame wrt to NED frame
    wvn_vel = zeros(3);
    wvn_vel[1] = -azdot*sin(gamma);
    wvn_vel[2] = gamdot;
    wvn_vel[3] = azdot*cos(gamma);
    wvn_body = Tr_bv*wvn_vel;

    ## Transport Rate
    #wne_ned[1] =  vm_ned[2]/RC.Lon;
    #wne_ned[2] = -vm_ned[1]/RC.Lat;
    #wne_ned[3] = -( vm_ned[2]*wgs_Pos.Latitude.Tan() )/RC.Lon;
    wne_ned = zeros(3);
    wne_body = Tr_bn*wne_ned;
    wne_eci = Tr_ib*wne_body;

    ## Velocity Angle rates
    wbv_body = zeros(3);
    wbv_body[1] = phidot*cos(alpha);
    wbv_body[2] = alphadot;
    wbv_body[3] = phidot*sin(alpha);

    ## Converting Earth Rotation Rate Vector to Body Frame
    wei_body = Tr_bi*wb_ei_eci;

    ## Computing Rate of body frame wrt inertial frame
    wbi_body = wbv_body + wvn_body + wne_body + wei_body;
    wbn_body = wbv_body + wvn_body;
    pp = wbn_body[1];
    qq = wbn_body[2];
    rr = wbn_body[3];

    ## Computing State Vector
    xo = [
            airframe.position.rm_eci;
            airframe.velocity.vm_eci;
         ];

    ## Computing Derivatives
    dx_pos = vm_eci;
    dx_vel = Tr_ib*(1.0/mass*( ftot_body + fgrav_body - cross( wbi_body, mass*vm_body ) ) );

    ## Derivative Vector
    dx = [dx_pos; dx_vel];

    ## Euler Integration
    xo = xo + dt*dx;
    time = time + dt;

    ## Updating States
    rm_eci = xo[1:3];
    vm_eci = xo[4:6];

    ## Position Update
    rm_ecef = Tr_ei*rm_eci;
    llh = ECEF2LLH(rm_ecef);
    lat = llh[1];
    lon = llh[2];
    alt = llh[3];
    Tr_ne = TR_NE(lat,lon);
    Tr_ei = TR_EI(time);
    Tr_ie = Tr_ei';
    pos = WGS_Pos(time, lat, lon, alt, rm_ecef, rm_eci, Tr_ei, Tr_ne);

    ## Velocity Update
    vm_ecef = Tr_ei*vm_eci - cross(wb_ei_ecef, rm_ecef);
    vm_ned = Tr_ne*vm_ecef;
    vmag = norm(vm_ned);
    vn = vm_ned[1];
    ve = vm_ned[2];
    vd = vm_ned[3];
    gamma = atan2( -vd, sqrt( vn*vn + ve*ve ) );
    azimuth = atan2( ve, vn );
    Tr_vn = TR_VN(gamma,azimuth);
    vel = Velocity_States(time, vmag, gamma, azimuth, vm_ned, vm_ecef, vm_eci);

    ## Updating Transformations
    Tr_nv = Tr_vn';
    Tr_en = Tr_ne';
    Tr_bn = Tr_bv*Tr_vn;
    Tr_nb = Tr_bn';
    Tr_eb = Tr_en*Tr_nb;
    Tr_be = Tr_eb';
    Tr_bi = Tr_be*Tr_ei;
    Tr_ib = Tr_bi';

    ## Euler Angle Updates
    eulervec = ComputeEuler( Tr_bn )
    roll = eulervec[1];
    pitch = eulervec[2];
    heading = eulervec[3];
    euler = Euler_Angles(roll,  pitch, heading, Tr_bn);

    ## Updating Quaternions
    quat_bi_body = QuatInit(Tr_bi);
    quat = Quaternions(quat_bi_body[1],
                       quat_bi_body[2],
                       quat_bi_body[3],
                       quat_bi_body[4] );

    ## Update Angle Rates
    angle_rates = Angle_Rates(wbi_body, wbn_body, wb_ei_eci, wb_ei_ecef, wne_eci)

    ## Updating the airframe structure
    airframe_out.time = time;
    airframe_out.position = pos;
    airframe_out.velocity = vel;
    airframe_out.euler = euler;
    airframe_out.angle_rates = angle_rates;
    airframe_out.quat_bi = quat;
    airframe_out.xo = xo;
    airframe_out.accel_meas = acc_meas_body;

    return airframe_out;
end
