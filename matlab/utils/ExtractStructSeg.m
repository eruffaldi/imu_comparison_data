%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function r = ExtractStructSeg(r,idxs)

flds = fields(r);

for i=1:length(flds)
    
    if isnumeric(r.(flds{i}))
        r.(flds{i}) = r.(flds{i})(idxs,:);
    end
end