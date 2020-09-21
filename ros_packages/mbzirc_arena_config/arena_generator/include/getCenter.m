function [ c ] = getCenter(p1, p2, p3, p4)

% Get arena center
dline1 = cross([p1;1], [p3;1]);
dline2 = cross([p2;1], [p4;1]);

c = linesIntersection(dline1, dline2);

c = normalizePoint(c);
c = c(1:2);

end
