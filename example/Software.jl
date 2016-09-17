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

#######################################################################
# Intialization
function Initialize_SW(time::Float64, DT::Float64, nav_state::NavStates)
    zz = 0.0;
    zvec = [0.0;0.0;0.0];
    dt = DT;
    count = UInt(0);
    mode = UInt(0);

    ## Motor Cmds
    motorcmd = MotorCmd( zz, zz, zz, zz );
    
    ## Software Telemetry
    swtelem = SoftwareTelem(time, nav_state, zz, zz, zz, zz, zz, zz);

    ## Software Message
    swMsg = SoftwareMsg( time, dt, zvec, zz, count, mode, swtelem, motorcmd );
    
    return swMsg;
end
    
#######################################################################
# Flight Control
@debug function Autopilot( nav_state::NavStates,
                    acc_cmds::Array{Float64,1},
                    softwareMsg::SoftwareMsg)

    ## Software Telemetry Variable
    softwareTM = softwareMsg;
    eta = softwareTM.eta;
    dz = softwareTM.dz;

    ## Constants
    keng = 6.11e-8;
    kmeng = 1.5e-9;
    Larm = 0.15; ## m
    mass = 0.5; ## kg
    II = [
        2.32  0.0    0.0
        0.0   2.32   0.0
        0.0   0.0    4.0
    ]; ## kg-m^2
    dt = softwareMsg.dt;
    
    ## Nav Estimates
    vmag = nav_state.velocity.vmag;
    gamma = nav_state.velocity.gamma;
    azimuth = nav_state.velocity.azimuth;
    roll = nav_state.euler.roll;
    pitch = nav_state.euler.pitch;
    heading = nav_state.euler.heading;
    Tr_bn = nav_state.euler.Tr_bn;
    Tr_nb = Tr_bn';
    vm_body = Tr_bn*nav_state.velocity.vm_ned;
    alpha = atan2( vm_body[3], vm_body[1] );
    beta = atan2( vm_body[2],vmag );

    println("gamma = ",gamma)
    println("azimuth = ",azimuth)
    println("roll = ",roll)
    println("pitch = ",pitch)
    println("heading = ",heading)
    println("beta = ",beta)
    println("alpha = ",alpha)
    println("vmag = ",vmag)


    ## Measurements
    acc_meas_body = nav_state.accel_meas;
    wb_bi_body = nav_state.rate_meas;
    
    ## Rate Meas
    wxmeas = wb_bi_body[1];
    wymeas = wb_bi_body[2];
    wzmeas = wb_bi_body[3];
    
    ## Accel Meas
    axmeas = acc_meas_body[1];
    aymeas = acc_meas_body[2];
    azmeas = acc_meas_body[3];
    println("acc_meas = ",acc_meas_body);
    
    ## Mixing Logic
    Larm = 0.15;
    r1_arm_body = [ Larm*cosd(45); -Larm*sind(45); 0.0];
    r2_arm_body = [ Larm*cosd(45);  Larm*sind(45); 0.0];
    r3_arm_body = [-Larm*cosd(45);  Larm*sind(45); 0.0];
    r4_arm_body = [-Larm*cosd(45); -Larm*sind(45); 0.0];
    
    rx1 = r1_arm_body[1];
    rx2 = r2_arm_body[1];
    rx3 = r3_arm_body[1];
    rx4 = r4_arm_body[1];

    ry1 = r1_arm_body[2];
    ry2 = r2_arm_body[2];
    ry3 = r3_arm_body[2];
    ry4 = r4_arm_body[2];
    
    AA = [
        -ry1*keng -ry2*keng -ry3*keng -ry4*keng
         rx1*keng  rx2*keng  rx3*keng  rx4*keng
        -kmeng     kmeng    -kmeng     kmeng     
        -keng     -keng     -keng     -keng
    ];

    A1 = [
    6.48063364957471e-9  -6.48063364957471e-9  -6.48063364957471e-9  6.48063364957471e-9 
    6.48063364957471e-9  6.48063364957471e-9   -6.48063364957471e-9  -6.48063364957471e-9
    -1.5e-9               1.5e-9               -1.5e-9                1.5e-9    
    -6.11e-8              -6.11e-8              -6.11e-8              -6.11e-8
    ];

    ## Acceleration Commands
    axcmd = acc_cmds[1];
    aycmd = acc_cmds[2];
    azcmd = acc_cmds[3];
    
    ## Acceleration Loop
    wn = 1.0*2.0*pi;
    Kp = wn;

    if abs(dz) > 1.0
        pcmd = Kp/dz*(aycmd - aymeas);
        qcmd = -Kp/dz*(axcmd - axmeas);
    else
        pcmd = Kp*(aycmd - aymeas);
        qcmd = -Kp*(axcmd - axmeas);
    end               
    
    println("azcmd = ",azcmd);
    println("azmeas = ",azmeas);
    
    dzdot = Kp*(azcmd - azmeas);
    dz = dz + dt*dzdot;

    if vmag < 50.0 
        heading_cmd = 0.0;
        rcmd = Kp*(heading_cmd - heading);
    else
        rcmd = cos(pitch)/cos(roll)*Kp*beta + aymeas*cos(pitch)/(vmag*cos(gamma)*cos(roll)) - wymeas*tan(roll) ;
    end

    wbcmd_body = [pcmd;qcmd;rcmd];
    
    ## Rate Loop
    Kpw = 0.3*2.0*pi;
    Kiw = 10.0;
    
    ## Moment Command
    mcmd_body = [0.0;0.0;0.0];
    mcmd_body = Kpw*II*(wbcmd_body - wb_bi_body) + Kiw*eta + cross( wb_bi_body, II*wb_bi_body );
    etadot = wbcmd_body - wb_bi_body;
    eta = eta + dt*etadot;

    println("wb_bi_body = ",wb_bi_body);
    println("etadot = ",etadot);
    println("eta = ",eta);
    println("dzdot = ",dzdot);
    println("dz = ",dz);

    ## Motor Commands
    ff = [mcmd_body; dz];
    cc = AA^-1*ff;

    w1 = sqrt( abs(cc[1]) );
    w2 = sqrt( abs(cc[2]) );
    w3 = sqrt( abs(cc[3]) );
    w4 = sqrt( abs(cc[4]) );

    ## Telemetry
    softwareTM.time = nav_state.time;
    softwareTM.dt = dt;
    softwareTM.eta = eta;
    softwareTM.dz = dz;
    softwareTM.mode = 0;
    softwareTM.softwareTM.time = time;
    softwareTM.softwareTM.navstates = nav_state;
    softwareTM.softwareTM.axcmd = axcmd;
    softwareTM.softwareTM.aycmd = aycmd;
    softwareTM.softwareTM.azcmd = azcmd;
    softwareTM.softwareTM.pcmd = pcmd;
    softwareTM.softwareTM.qcmd = qcmd;
    softwareTM.softwareTM.rcmd = rcmd;
    softwareTM.motorcmd = MotorCmd( w1, w2, w3, w4 );

    Pause()
    
    return softwareTM;
end


#######################################################################
# Software Update
@debug function Update_SW(motorcmd::MotorCmd, nav_state::NavStates, swMsg::SoftwareMsg)

    swOut = swMsg;
    
    ## Update
    swOut.time = swOut.time + swOut.dt;
    swOut.count = swOut.count + 1;
        
    ## Position and Veloity Command
    latcmd = 0.0;
    loncmd = 0.0;
    altcmd = 1.0;
    poscmd = InitPos(swOut.time, latcmd, loncmd, altcmd);

    vmagcmd = 0.0;
    gamcmd = 0.0;
    azcmd = 0.0;
    velcmd = InitVel(vmagcmd, gamcmd, azcmd, poscmd);

    roll = nav_state.euler.roll;
    pitch = nav_state.euler.pitch;
    heading = nav_state.euler.heading;
    Tr_bn = TR_BN(roll, pitch, heading);

    Tr_ne = poscmd.Tr_ne;
    Tr_en = Tr_ne';

    poserr_ned = Tr_ne*(poscmd.rm_ecef - nav_state.wgs_pos.rm_ecef);
    velerr_ned = velcmd.vm_ned - nav_state.velocity.vm_ned;

    wn = 0.2;
    zz = 1.0;
    Kp = wn^2;
    Kd = 2.0*wn*zz;
    acc_cmd_ned = Kp*poserr_ned + Kd*velerr_ned;

    acc_cmd_body = Tr_bn*acc_cmd_ned;
    
    ## Autopilot
    swOut = Autopilot( nav_state,
               acc_cmd_body,
               swMsg )

    return swOut;
end
