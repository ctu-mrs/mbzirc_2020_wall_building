function [ out ] = linesAngle(line1, line2)

  l1 = [line1(2) -line1(1)]';
  l2 = [line2(2) -line2(1)]';
  
  out = acos(dot(l1, l2)/(norm(l1)*norm(l2)));

end

