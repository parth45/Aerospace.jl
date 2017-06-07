############################################################################
#
#  Software.jl
#
#  Purpose:    Software file to control a quadrotor
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################
include("Guidance.jl")

#######################################################################
# Intialization
function Initialize_SW(time::Float64, DT::Float64, nav_state::NavStates)
    zz = 0.0;
    zi = UInt(0);
    zvec = [0.0;0.0;0.0];
    dt = DT;
    count = UInt(0);
    mode = UInt(0);

    ## Controls Initialization
    throttle = 0.0;
    alpha = 0.0;
    alphadot = 0.0;
    phi = 0.0;
    phidot = 0.0;
    controls = Controls(throttle,alpha,alphadot,phi,phidot);

    ## Guidance Initialization
#    num_waypoints = 3;
#    GuidInit(wyptlist, num_waypoints)

    ## Software Message
    tgo = 999999.0;
    missvec = [9999999.0;9999999.0;9999999.0];
    miss = norm(missvec);
    swMsg = SoftwareMsg( time, dt, mode, controls, nav_state, zi, zz, zz, zz, zz, zz, tgo, miss, missvec, zz );

    return swMsg;
end

#######################################################################
# Flight Control
function FlightControl( softwareMsg::SoftwareMsg, apcmd::Float64, phicmd::Float64 )

    ## Software Telemetry Variable
    swOut = softwareMsg
    time = swOut.time;
    dt =  swOut.dt;

    ## Control Variables
    alpha = swOut.controls.alpha;
    alphadot = swOut.controls.alphadot;
    phi = swOut.controls.phi;
    phidot = swOut.controls.phidot;
    throttle = swOut.controls.throttle;

    ## IMU Measurements
    axmeas = swOut.navstates.accel_meas[1];
    aymeas = swOut.navstates.accel_meas[2];
    azmeas = swOut.navstates.accel_meas[3];

    # Alpha Gains
    Kpa = 0.20;
    Kpad = 0.3;

    # Roll Gain
    Kr = 0.30;

    # Z Acceleration Error
    alphacmd = -Kpa*(apcmd  - azmeas);

    # Alpha Response
    alphadot = Kpad*(alphacmd - alpha);
    tmpalpha = alpha + dt*alphadot;

    # Limits on the Angle of Attack
    if (tmpalpha*57.3 < -6.0 )
        alpha = -6.0/57.3;
    elseif (tmpalpha*57.3 > 9.0 )
        alpha = 9.0/57.3;
    else
        alpha = tmpalpha;
    end

    # Phi Response
    phidot = Kr*(phicmd - phi);
    tmpphi = phi + dt*phidot;

    # Limits on the Roll Angle
    if ( tmpphi*57.3 > 80.0)
        phi = 80.0/57.3;
    elseif ( tmpphi*57.3 < -80.0)
        phi = -80.0/57.3;
    else
        phi = tmpphi;
    end

    swOut.controls.alpha = alpha;
    swOut.controls.alphadot = alphadot;
    swOut.controls.alphadot = phi;
    swOut.controls.alphadot = phidot;

    return swOut
end

#######################################################################
function Steering(nav_state::NavStates, acc_cmd_ned::Array{Float64,1})

    # Computing NED to Body DCM
    Tr_BN = nav_state.euler.Tr_bn;
    roll = nav_state.euler.roll;
    pitch = nav_state.euler.pitch;
    heading = nav_state.euler.heading;

    # Computing Non-Rolled NED To Body DCM
    Tr_BN_NR = TR_BN(0.0, pitch, heading);

    # Acceleration Commands Body
    acc_cmd_body = Tr_BN*acc_cmd_ned;
    acc_cmd_bodyNR = Tr_BN_NR*acc_cmd_ned;

    # Acceleration Commands Non Rolled
    ay_cmd_ned_nr = acc_cmd_bodyNR[2];
    az_cmd_ned_nr = acc_cmd_bodyNR[3];

    # Zero Denominator Protection
    if ( abs(az_cmd_ned_nr) < 0.00000001 )
        phicmd = 0.5*eta_turn_cmd;
    else
        phicmd = atan(ay_cmd_ned_nr/abs(az_cmd_ned_nr + 0.001) );
    end

    # Phi Limiting
    philim = 60.0*d2r;
    phicmd = Bound(phicmd, -philim, philim);

    # Pitch Acceleration Command
    apcmd = acc_cmd_body[3];

    return  [apcmd; phicmd];
end

#######################################################################
function SpeedControl(nav_state::NavStates, vcmd_coop::Float64, softwareMsg::SoftwareMsg )

    ## Parameter Initialization
    vm_ned = nav_state.velocity.vm_ned;
    gamma = nav_state.velocity.gamma;
    alt = nav_state.wgs_pos.altitude;
    Vmag = nav_state.velocity.vmag;
    vel_err_int = softwareMsg.vel_err_int;
    dt50 = softwareMsg.dt;
    Throttle = softwareMsg.controls.throttle;

    ## Atmosphere Model
    atmdata = Atmosphere(alt, Vmag);
    Press = atmdata[1];
    Temp = atmdata[2];
    rho = atmdata[3];
    aa = atmdata[4];
    Mach = atmdata[5];
    qbar = atmdata[6];

    ## Mach Command converted to speed command
    machcmd = 0.7;
    vcmd = machcmd*aa + vcmd_coop;

    ## Velocity Error and Integrator
    verr = (vcmd - Vmag);
    tmp2 = vel_err_int + dt50*verr;

    ## PI Control to compute Throttle Command
    Kpv = 1.5;
    Kiv = 0.66;
    tmp = Kpv*verr + Kiv*tmp2;

    if ( tmp > 100.0 )
        tmp = 100.0;
    elseif ( tmp < 0.0)
        tmp = 0.0;
    else
        Throttle = tmp;
        vel_err_int = tmp2;
    end

    ## Bounding Throttle to be between 0 and 100
    Throttle = Bound(Throttle, 0.0, 100.0);

    return Throttle;
end

#######################################################################
# Software Update
function Update_SW(swMsg::SoftwareMsg)

    swOut = swMsg;
    alpha  = swOut.controls.alpha;
    phi  = swOut.controls.phi;
    gamma  = swOut.navstates.velocity.gamma;
    azimuth  = swOut.navstates.velocity.azimuth;

    ## Update
    swOut.time = swOut.time + swOut.dt;

    ## Acceleration Commands
    acc_cmd_ned = zeros(3);

    ## Guidance Section
    alt_cmd = 4000.0;
    eta_turn_cmd = 0.0;
    eta_dive_cmd = AltitudeControl(swOut.navstates, alt_cmd);
    println(eta_dive_cmd)

    ## Velocity Commands in Steering Frame (Velocity Frame)
    acc_cmd_vel = zeros(3);
    acc_cmd_vel[1] = 0.0;
    acc_cmd_vel[2] = eta_turn_cmd;
    acc_cmd_vel[3] = eta_dive_cmd;

    Tr_vn = TR_VN(gamma, azimuth);
    Tr_nv  = Tr_vn';
    acc_cmd_ned = Tr_nv*acc_cmd_vel;
    acc_cmd_ned[3] = acc_cmd_ned[3]  - 9.80665;

    ## Steering Function
    steerout = Steering(swOut.navstates, acc_cmd_ned);
    apcmd = steerout[1];
    phicmd = steerout[2];

    ## Flight Controls
    throttle = SpeedControl(swOut.navstates, 0.0, swOut);
    FlightControl( swOut, apcmd, phicmd );

    swOut.controls.throttle = throttle;
    return swOut;
end
