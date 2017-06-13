function newfld = vicfldmap(vic_fld,bodypart)

if strcmp(bodypart,'Up')
    switch vic_fld
        case 'UVRKOKR'
            newfld = 'cam_imu';
        case 'ZOBROGT'
            newfld = 'L_lower_arm';
        case 'TNRISWC'
            newfld = 'Trunk';
        case 'XADLTMY'
            newfld = 'R_upper_arm';
        case 'PPEIVDY'
            newfld = 'R_lower_arm';
        case 'GWAXWGN'
            newfld = 'R_Shoulder';
        case 'TAHWRAG'
            newfld = 'L_upper_arm';
        case 'GDVFNDU'
            newfld = 'L_Shoulder';
        case 'LVQBICJ'
            newfld = 'Pelvis';
        case 'Back'
            newfld = 'Back_vic';
        case 'Pelvis'
            newfld = 'Pelvis_vic';
        otherwise
            newfld = [];
    end
end