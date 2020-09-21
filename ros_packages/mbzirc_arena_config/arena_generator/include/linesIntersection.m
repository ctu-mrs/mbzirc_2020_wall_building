function [ out ] = linesIntersection(line1, line2)

    out = normalizePoint(cross(line1, line2));
        
end

