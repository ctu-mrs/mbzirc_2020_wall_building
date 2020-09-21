function [ out ] = segmetLineIntersection(segment, line)

    segment

    % calculate the line of the segment
    segmentLine = cross(segment(:, 1), segment(:, 2));
    
    % calculate the intersection
    intersection = linesIntersection(segmentLine, line);

    % make the segment vector
    segmentVector = segment(:, 2) - segment(:, 1);
    
    % calculate alpha
    alpha = (intersection - segment(:, 1)) ./ segmentVector;
    alpha = alpha(find(~isnan(alpha)));
    alpha = alpha(find(~isinf(alpha)));
    alpha = alpha(1);
    
    if (alpha >= 0 && alpha <= 1)
       out = intersection; 
    else if (alpha == inf)
       out = segment(:, 2);
    else
       out = nan;
    end
end

