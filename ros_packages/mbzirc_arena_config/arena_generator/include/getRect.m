function [ p1, p2, p3, p4 ] = getRect(c, ang, l, w)

  ca = cos(ang);
  sa = sin(ang);

  R = [ca sa 0; -sa ca 0; 0 0 1];

  lh = l/2;
  wh = w/2;

  bl = R*[-lh;-wh;1];
  br = R*[lh;-wh;1];
  tr = R*[lh;wh;1];
  tl = R*[-lh;wh;1];

  p1 = c + [bl(1:2);0];
  p2 = c + [br(1:2);0];
  p3 = c + [tr(1:2);0];
  p4 = c + [tl(1:2);0];

end
