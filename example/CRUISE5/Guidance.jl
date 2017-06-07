type GuidType
    guidance_mode::UInt
    num_waypts::UInt
    current_wypt::UInt
    WaypointList::Array{WGS_Pos,1}
    tgo::Float64
    miss::Float64
    miss_vec_ned::Array{Float64,1}
    eta_dive_cmd::Float64
    eta_turn_cmd::Float64
    alt_cmd::Float64
    gamma_cmd::Float64
    Mach_cmd::Float64
end
################################################################################
## Cooperative Formation
function GuidInit(wyptlist::Array{WGS_Pos,1}, num_waypoints::Float64)
    guidance_mode = 0;
    current_wypt = 1;
    tgo = 99999.0;
    miss = 9999.0;
    miss_vec_ned = zeros(3);
    eta_dive_cmd = 0.0;
    eta_turn_cmd = 0.0;
    alt_cmd = 0.0;
    gamma_cmd = 0.0;
    Mach_cmd = 0.0;

    Guidance_Data = GuidType(guidance_mode, num_waypoints, current_wypt, wyptlist,
                             tgo, miss, miss_vec_ned, eta_turn_cmd, eta_dive_cmd,
                             alt_cmd, gamma_cmd, Mach_cmd );

    return Guidance_Data;
end


################################################################################
## Cooperative Formation
function CoopFormation(nav_state::NavStates, guidance_data::GuidType, SS::Array{Float64,2},
    missile_id::UInt,  salvo_size::UInt, message_list::Array{CoopFormation,1})

    # Computing NED to Body DCM
    Tr_BN = nav_state.euler.Tr_bn;
    Tr_NE = nav_state.position.Tr_ne;
    rm_ecef = nav_state.position.rm_ecef;
    vm_ecef = nav_state.velocity.vm_ecef;
    nn = length(message_list);

    ## Clearing the summation variables
    sumvec = zeros(3);
    sum = 0;
    ii = missile_id;
    icoop_flag = true;

    ## Gravity Vector in NED frame
    tmpvec = zeros;
    tmpvec[3] = 1.0;
    gvec_LL[1] = 0.0;
    gvec_LL[2] = 0.0;
    gvec_LL[3] = go;

    rm_ned = Tr_NE*rm_ecef;
    vm_ned = Tr_NE*vm_ecef;

    ## Computing the Guidance Command
    jj = 0;

    for jj = 1:NN

        ## Determine Communication Topology
        if ( message_list[jj].time >= 0 )
            SS[jj] = 1;
        else
            SS[jj] = 0;
        end

        ## For right now, Missile 0 is the leader.
        if( jj == 1 )
            rm_lead_ecef = message_list[jj].rm_ecef;
            vm_lead_ecef = message_list[jj].vm_ecef;
            rm_lead_ned = Tr_NE*rm_lead_ecef;
            vm_lead_ned = Tr_NE*vm_lead_ecef;

            ## Unit Velocity Vector
            unit_vmlead_ned = vm_lead_ned/norm(vm_lead_ned);

            ## Gamma and Azimuth of Lead Vector
            gamma_lead = atan2(-unit_vmlead_ned[3], sqrt( unit_vmlead_ned[1]^2 + unit_vmlead_ned[2]^2 ));
            azimuth_lead = atan2( unit_vmlead_ned[1], unit_vmlead_ned[0] );

            ## NED to Velocity Frame Transformation Matrix
            Roll_lead = 0.0;
            Tr_VN_LEAD = TR_BN(Roll_lead, gamma_lead, azimuth_lead );
            Tr_NV_LEAD = Tr_VN_LEAD';

            #delta_vel[0] = 0.0;
            #delta_vel[1] = YY(jj) - YY(missile_number);
            #delta_vel[2] = 0.0;

            delta_vel = zeros(3);
            delta_vel[1] = 0.0;
            delta_vel[2] = 200.0*sin(0.20*guidtime + (ii-1)*2.0*MC::PI/(salvo_size - 1));
            delta_vel[3] = 200.0*cos(0.20*guidtime + (ii-1)*2.0*MC::PI/(salvo_size - 1));
            delta_ned = Tr_NV_LEAD*delta_vel;

            wn = 0.5;
            zz = 0.70;
            Kpf = wn*wn;
            Kvf = 2.0*wn*zz;
            sumvec = sumvec + SS[jj]*( Kpf*(rm_lead_ned - rm_ned - delta_ned ) + Kvf*(vm_lead_ned - vm_ned) );

        end

        jj = jj + 1;
    end

    ## Guidance Command
    acc_cmd_ned = sumvec - gvec_LL;
    acc_cmd_vel = Tr_VN*acc_cmd_ned;

    speed_accel_cmd = acc_cmd_vel[0];
    eta_turn_cmd = acc_cmd_vel[1];
    eta_dive_cmd = acc_cmd_vel[2];

    turn_lim = 3.0*go;
    speed_accel_cmd = Bound(speed_accel_cmd, -turn_lim, turn_lim );

    vcmd_coop = vcmd_coop + dt50*speed_accel_cmd;
    eta_turn_cmd = Bound(eta_turn_cmd, -turn_lim, turn_lim );
    eta_dive_cmd = Bound(eta_dive_cmd, -turn_lim, turn_lim );

end

################################################################################
## Altitude Command
function AltitudeControl(nav_state::NavStates, alt_cmd::Float64)

    ## Parameter Computation
    # Computing NED to Body DCM
    Tr_BN = nav_state.euler.Tr_bn;
    Tr_NE = nav_state.wgs_pos.Tr_ne;
    rm_ecef = nav_state.wgs_pos.rm_ecef;
    vm_ecef = nav_state.velocity.vm_ecef;
    vm_ned = Tr_NE*vm_ecef;

    gamma = atan2( -vm_ned[3], sqrt( vm_ned[1]^2 + vm_ned[2]^2 ) );
    alt = nav_state.wgs_pos.altitude;
    Vmag = nav_state.velocity.vmag;

    ## Altitude Control Gains
    Kalt = 0.210;

    ## Altitude Error
    alt_err = alt_cmd - alt;
    println("alt_err = ", alt_err)

    ## Gamma Command
    gammac = Kalt/Vmag*alt_err;

    gamlim = 10.0*d2r;
    gammac = Bound(gammac, -gamlim, gamlim);

    ## Gamma Control Gains
    wn = 1.2;
    zz = 0.70;
    Kvf = 2*zz*wn;
    Kpf = wn*wn;

    ## Eta Dive Command
    eta_dive_cmd = -Kpf*Vmag*(gammac - gamma);
    eta_dive_cmd = Bound(eta_dive_cmd, -1.5*go, 1.5*go );

    return eta_dive_cmd;
end
