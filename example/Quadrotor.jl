############################################################################
#
#  Quadrotor.jl
#
#  Purpose:    Top level quadrotor file
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################
# Mass Properties
#    This function provides the mass properties for a simple quadrotor
function Mass_Props()
    mass = 0.5;  # Kg

    II = [
        2.32  0.0    0.0
        0.0   2.32   0.0
        0.0   0.0    4.0
    ]*10.0^-3; ## kg-m^2

    cg_vec_body = [0;0;0.0];

    massprops = MassProps(mass,II,cg_vec_body);

    return massprops;
end

############################################################################
# Initialize Quadrotor Structure
function QuadInit( time::Float64, dt::Float64, position_init::WGS_Pos, vel_init::Velocity_States, euler_init::Euler_Angles, rate_init::Angle_Rates )

    # Mass Properties
    massprop = Mass_Props();

    # Airframe Initialization
    quad_airframe = Airframe_Initialize( time, time, dt, position, velocity, euler, angle_rates, massprop );

    # Navigation Message Intialization
    zvec = [0.0;0.0;0.0];
    nav_state = NavStates( time, position, velocity, rate_init, zvec, zvec, euler, quad_airframe.quat_bi );

    # Motor Initialization
    wn = 220.0;
    zeta = 0.707;
    rpm_pos_lim = 100000.0*2.0*pi;
    rpm_neg_lim = 0.0;
    rpm_rate_pos_lim = 250.0;
    rpm_rate_neg_lim = -250.0;

    motor1 = Initialize_Act(dt, wn, zeta,
                            rpm_pos_lim,
                            rpm_neg_lim,
                            rpm_rate_pos_lim,
                            rpm_rate_neg_lim
                            );

    motor2 = Initialize_Act(dt, wn, zeta,
                            rpm_pos_lim,
                            rpm_neg_lim,
                            rpm_rate_pos_lim,
                            rpm_rate_neg_lim
                            );

    motor3 = Initialize_Act(dt, wn, zeta,
                            rpm_pos_lim,
                            rpm_neg_lim,
                            rpm_rate_pos_lim,
                            rpm_rate_neg_lim
                            );

    motor4 = Initialize_Act(dt, wn, zeta,
                            rpm_pos_lim,
                            rpm_neg_lim,
                            rpm_rate_pos_lim,
                            rpm_rate_neg_lim
                            );


    # Motor Array
    motor_array = Motors(motor1, motor2, motor3, motor4);

    # Initial Motor Commands
    zz = 5000.0;
    motorCmd = MotorCmd(zz, zz, zz, zz);

    ## Software Initialization
    swMsg = Initialize_SW(time, dt, nav_state);

    # Creating Quadrotor Structure
    quad_rotor = Quadrotor( time, dt, quad_airframe, motor_array, motorCmd, swMsg);

    return quad_rotor;
end


############################################################################
# Quadrotor Update
function QuadUpdate(quad::Quadrotor)

    quad_rotor = quad;

    ## Extracting Motor Commands
    motorcmd = quad_rotor.motorCmd;

    ## Taking Measurements
    time = quad_rotor.time = quad_rotor.dt;
    quad_rotor.time = time;
    position = quad_rotor.airframe.position;
    velocity = quad_rotor.airframe.velocity;
    rate = quad_rotor.airframe.angle_rates;
    rate_meas = quad_rotor.airframe.angle_rates.wb_bi_body;
    accel_meas = quad_rotor.airframe.accel_meas;
    euler = quad_rotor.airframe.euler;
    quat_bi = quad_rotor.airframe.quat_bi;
    nav_state = NavStates( time, position, velocity, rate, rate_meas, accel_meas, euler, quat_bi );

    # Parameter Update
    motor_array = quad.motors;
    motor1 = motor_array.motor1;
    motor2 = motor_array.motor2;
    motor3 = motor_array.motor3;
    motor4 = motor_array.motor4;

    # Software Update
    swMsg = Update_SW(motorcmd, nav_state, quad_rotor.software);
    quad_rotor.software = swMsg;
    quad_rotor.motorCmd = quad_rotor.software.motorcmd;

    # Actuator Update
    motor_cmds = swMsg.motorcmd;
    motor1 = Update_Act( motor1, motor_cmds.motor1_cmd );
    motor2 = Update_Act( motor2, motor_cmds.motor2_cmd );
    motor3 = Update_Act( motor3, motor_cmds.motor3_cmd );
    motor4 = Update_Act( motor4, motor_cmds.motor4_cmd );
    motor_array.motor1 = motor1;
    motor_array.motor2 = motor2;
    motor_array.motor3 = motor3;
    motor_array.motor4 = motor4;
    quad_rotor.motors = motor_array;

    # Quadrotor Forces and Moments
    QuadForceArray = Quad_Forces( motor_array );
    ftot_rotor_body = QuadForceArray.f1body +
        QuadForceArray.f2body +
        QuadForceArray.f3body +
        QuadForceArray.f4body;
    mtot_rotor_body = QuadForceArray.m1body +
        QuadForceArray.m2body +
        QuadForceArray.m3body +
        QuadForceArray.m4body;

    # Airframe Update
    quad_rotor.airframe = Airframe_Update(quad_rotor.airframe, ftot_rotor_body, mtot_rotor_body );

    return quad_rotor;
end
