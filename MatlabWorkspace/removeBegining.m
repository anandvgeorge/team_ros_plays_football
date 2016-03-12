% team ros for soccer robot project on vrep 
% Romain Chiappinelli A0145192H
% 12.03.16
function [t, d] = removeBegining(t,d, epsilon, n)
% removeBegining()remove the beginning part of a signal starting around 0
% t: time, d: signal magnitude corresponding at time t,
% epsilon: remove all the part of t and d while |d|<epsilon
i=1;
while(abs(d(i))<epsilon)
    i=i+1;
end
t = t(i-n:end)-t(i-n);
d = d(i-n:end);
% return t, d
