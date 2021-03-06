function dfz = dfuz(z_i,h_i,delta_i,z_min,z_max,delta_min,delta_max)
%DFUZ
%    DFZ = DFUZ(Z_I,H_I,DELTA_I,Z_MIN,Z_MAX,DELTA_MIN,DELTA_MAX)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    05-Nov-2018 21:05:56

t2 = z_i-z_min;
t3 = z_min-z_max;
dfz = 1.0./t3.^4.*(z_i.*2.0-z_min.*2.0).*(t2.^2-t3.^2).*(2.0./3.0);
