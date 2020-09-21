function [ segmentBuffer ] = plotSegment( segmentBuffer, x, y, color, width )

    segment.x = x;
    segment.y = y;
    segment.color = color;
    segment.width = width;

    segmentBuffer = [segmentBuffer, segment];
    
end

