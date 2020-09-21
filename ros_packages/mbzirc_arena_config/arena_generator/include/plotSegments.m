function [] = plotSegments(hfig, segmentBuffer, H)

    figure(hfig);

    for i=1:size(segmentBuffer, 2)
        
        if (nargin == 3)
            segmentBuffer(i).x = normalizePoint(H * segmentBuffer(i).x);
            segmentBuffer(i).y = normalizePoint(H * segmentBuffer(i).y);
        end
        
        if (size(segmentBuffer(i).x, 1) == 3)
        
            plot([segmentBuffer(i).x(1) segmentBuffer(i).y(1)], [segmentBuffer(i).x(2), segmentBuffer(i).y(2)], 'Color', segmentBuffer(i).color, 'LineWidth', segmentBuffer(i).width)      
       
        else if (size(segmentBuffer(i).x, 1) == 4)
                                       
            plot3([segmentBuffer(i).x(1) segmentBuffer(i).y(1)], [segmentBuffer(i).x(2), segmentBuffer(i).y(2)], [segmentBuffer(i).x(3), segmentBuffer(i).y(3)], 'Color', segmentBuffer(i).color, 'LineWidth', segmentBuffer(i).width)      
        end
    end

end

