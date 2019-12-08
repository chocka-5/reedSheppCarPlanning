function ang = ang_wrap(ang)

while ang <= pi
    ang = ang + 2*pi;
end

while ang > pi
    ang = ang - 2*pi;
end
