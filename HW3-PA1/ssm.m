function M = ssm(vect)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
M = [0 -vect(3) vect(2);vect(3) 0 -vect(1);-vect(2) vect(1) 0];
end