function angle_difference = angle_subtract(angle1,angle2)
%ANGLE_SUBTRACT Gives the angle difference between two angles on the circle

raw_difference = angle1 - angle2;
angle_difference = mod(raw_difference+pi,2*pi)-pi;

end