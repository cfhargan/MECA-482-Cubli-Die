l = 0.085;
lb = 0.075;
mb = 0.419;
mw = 0.204;
Ib = 3.34/(10*10*10);
Iw = 0.57/(10*10*10);
Cb = 0; %1.02/(10*10*10);
Cw = 0; %0.05/(10*10*10);
g = 9.8;
Km = 1; %0.0251;

A = [0 1 0;
    (mb*lb+mw*l)*g/(Ib+mw*l*l)   -Cb/(Ib+mw*l*l)   Cw/(Ib+mw*l*l);
    -1*(mb*lb+mw*l)*g/(Ib+mw*l*l)   Cb/(Ib+mw*l*l)   -Cw*(Ib+Iw+mw*l*l)/(Iw*(Ib+mw*l*l))]

B = [0;
    -Km/(Ib+mw*l*l);
     Km*(Ib+Iw+mw*l*l)/(Iw*(Ib+mw*l*l))]


Q = diag([10^-2 10^-2 10^-8]);
R = 0.001;
[K,S,e] = lqr(A,B,Q,R)

