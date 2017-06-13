%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function myparent = selparent(fldname)

switch fldname
    
    case {'Static_glob_cam','Trunk'}
        myparent = 'None';
        
    case{'L_Shoulder','R_Shoulder'}
        myparent = 'Trunk';
        
    case 'L_upper_arm'
        myparent = 'L_Shoulder';
        
    case 'L_lower_arm'
        myparent = 'L_upper_arm';
            
    case 'R_upper_arm'
        myparent = 'R_Shoulder';
        
    case 'R_lower_arm'
        myparent = 'R_upper_arm';
end