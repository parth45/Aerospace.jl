using SymPy

wmotor1 = Sym("wmotor1");
wmotor2 = Sym("wmotor2");
wmotor3 = Sym("wmotor3");
wmotor4 = Sym("wmotor4");

## Propeller Forces
keng = 6.11e-3;
f1_body = [0;0;-keng*wmotor1];
f2_body = [0;0;-keng*wmotor2];
f3_body = [0;0;-keng*wmotor3];
f4_body = [0;0;-keng*wmotor4];

## Propeller Moments
kmeng = 1.5e-4;
m1_body = [0;0;-kmeng*wmotor1];
m2_body = [0;0; kmeng*wmotor2];
m3_body = [0;0;-kmeng*wmotor3];
m4_body = [0;0; kmeng*wmotor4];

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

mtot = m1_body + m2_body + m3_body + m4_body;
ftot = f1_body + f2_body + f3_body + f4_body;

AA = jacobian([mtot;ftot[3]], [wmotor1, wmotor2, wmotor3, wmotor4]);
