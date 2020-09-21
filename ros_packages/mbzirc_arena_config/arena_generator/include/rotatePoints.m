function [ pts ] = rotatePoints(pts, c, ang)

  ca = cos(ang);
  sa = sin(ang);

  R = [ca sa 0; -sa ca 0; 0 0 1];

  pts = pts - c;

  pts = [pts; ones(1, size(pts,2))];

  pts = R*pts;
  
  pts = pts + [c;0];

end
