############################################################################
#
#  Actuator.jl
#
#  Purpose:    Nonlinear Actuator model
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################

using Aerospace

############################################################################
## Actuator Initialization
function Initialize_Act(dt::Float64,
                        wn::Float64,
                        zeta::Float64,
                        pos_lim_pos::Float64,
                        pos_lim_neg::Float64,
                        rate_lim_pos::Float64,
                        rate_lim_neg::Float64
                        )

    ## Time, Position, and Rate Initialization
    time = 0.0;
    position = 0.0;
    rate = 0.0;

    ## Creating Actuator Structure
    actout = Actuator(time, dt, wn, zeta, position, rate,
                      pos_lim_pos,
                      pos_lim_neg,
                      rate_lim_pos,
                      rate_lim_neg
                      );

    return actout;
end

############################################################################
## Actuator Update
function Update_Act(act::Actuator, position_cmd::Float64)   

    act_out = act;
    
    ## Variable Initialization
    x1 = act.position;
    position = x1;
    x2 = act.rate;
    rate = x2;
    wn = act.wn;
    zeta = act.zeta;
    pos_lim_pos = act.pos_lim_pos;
    pos_lim_neg = act.pos_lim_neg;
    rate_lim_pos = act.pos_lim_pos;
    rate_lim_neg = act.rate_lim_neg;
    dt = act.dt;
    time = act.time;
    
    ## Limiting Command
    if (position_cmd > pos_lim_pos ) 
        position_cmd = pos_lim_pos;
    end
    
    if (position_cmd < pos_lim_neg ) 
        position_cmd = pos_lim_neg;
    end

    wn2 = wn*wn;
    d2 = 2.0*wn*zeta;
    
    ## Integrating to get X2
    dx2 = wn2*(position_cmd - position) - d2*rate;
    x2p = x2 + dt*dx2;

    ## Limiting Rate
    if (x2p > rate_lim_pos  ) 
        x2p = rate_lim_pos;
    end
    
    if (x2p < rate_lim_neg ) 
        x2p = rate_lim_neg;
    end

    ## Integrating to get X1   
    dx1 = x2p;
    x1 = x1 + dt*dx1;
    
    ## Limiting Command
    if (x1 > pos_lim_pos ) 
        x1 = pos_lim_pos;
        x2 = 0.0;    
    elseif (x1 < pos_lim_neg ) 
        x1 = pos_lim_neg;
        x2 = 0.0;
    else
        x2 = x2p;
    end
    
    ## Updating Time and Member data
    time = time + dt;
    position = x1;
    rate = x2;
    act_out.time = time;
    act_out.position = position;
    act_out.rate = rate;

    return act_out;
end


