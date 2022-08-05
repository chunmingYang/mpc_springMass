clc
clear all
close all

ts = 0.01;
iter = 200;
x = 0.1;          % state 1 begin - position
xf = 0.2;


t0 = 0;
tf = ts*iter;
y0 = x;   % position start position
yf = xf;  % position end position

a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
yhis = [];

for i = 1 : iter
    y = a0 + a1*(i*ts) + a2*(i*ts)*(i*ts) + a3*(i*ts)*(i*ts)*(i*ts);
    yhis(i) = y;
end

plot(1:iter, yhis)

% t0 = 0;
% tf = ts*iter;
% y0 = x;   % position start position
% yf = xf;  % position end position
% 
% a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
% a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
% a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
% a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));

% y = a0 + a1*d->time + a2*d->time*d->time + a3*d->time*d->time*d->time;