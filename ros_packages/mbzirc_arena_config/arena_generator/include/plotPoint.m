function [pointBuffer] = plotPoint( pointBuffer, x, color, size )

    if (x == nan)
       exit; 
    end
    
    point.coordinates = x;
    point.color = color;
    point.size = size;
    
    pointBuffer = [pointBuffer, point];
    
end

