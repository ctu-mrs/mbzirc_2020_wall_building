function [] = plotPoints( hfig, pointBuffer, H )

    figure(hfig);
    
    for i=1:size(pointBuffer, 2)
        
        if (nargin == 3)
            pointBuffer(i).coordinates = normalizePoint(H * pointBuffer(i).coordinates);
        end
        
        scatter(pointBuffer(i).coordinates(1), pointBuffer(i).coordinates(2), pointBuffer(i).size, pointBuffer(i).color, "filled")
    end
end

