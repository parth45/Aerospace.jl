############################################################################
#
#  Quad_Forces.jl
#
#  Purpose:    Computes the forces and moments due to rotors.
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################

function Quad_Forces(motor_array::Motors)

    ## Motor Rate (rad/s)
    wmotor1 = motor_array.motor1.position;
    wmotor2 = motor_array.motor2.position;
    wmotor3 = motor_array.motor3.position;
    wmotor4 = motor_array.motor4.position;
    
    ## Propeller Forces
    keng = 6.11e-8;
    f1_body = [0;0;-keng*wmotor1^2];
    f2_body = [0;0;-keng*wmotor2^2];
    f3_body = [0;0;-keng*wmotor3^2];
    f4_body = [0;0;-keng*wmotor4^2];

    ## Propeller Moments
    kmeng = 1.5e-9;
    m1_body = [0;0;-kmeng*wmotor1^2];
    m2_body = [0;0;kmeng*wmotor2^2];
    m3_body = [0;0;-kmeng*wmotor3^2];
    m4_body = [0;0;kmeng*wmotor4^2];

    ## Moment arm from CG to Propeller Hub;
    Larm = 0.15;
    r1_arm_body = [ Larm*cosd(45); -Larm*sind(45); 0.0];
    r2_arm_body = [ Larm*cosd(45);  Larm*sind(45); 0.0];
    r3_arm_body = [-Larm*cosd(45);  Larm*sind(45); 0.0];
    r4_arm_body = [-Larm*cosd(45); -Larm*sind(45); 0.0];

    ## Total Moment due to Motors
    m1_body = m1_body + cross(r1_arm_body, f1_body);
    m2_body = m2_body + cross(r2_arm_body, f2_body);
    m3_body = m3_body + cross(r3_arm_body, f3_body);
    m4_body = m4_body + cross(r4_arm_body, f4_body);

    println(m1_body)
    println(m2_body)
    println(m3_body)
    println(m4_body)
    PauseJulia()

    ## Creating Quad Forces Structure
    quadforces = QuadForces(f1_body,
                            f2_body,
                            f3_body,
                            f4_body,
                            m1_body,
                            m2_body,
                            m3_body,
                            m4_body);
    
    
    return quadforces;
end
