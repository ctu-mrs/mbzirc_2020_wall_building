function [ out ] = normalizePoint( in )

    out = in;

    if (~isnan(in(end)) && in(end)~=0)
       out = in ./ in(end);
    end

end

