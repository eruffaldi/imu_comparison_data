%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [ang,pos,rotax] = model_pos_ang(time,ref,camref,Q,par,side,ploten)

if strcmp(side,'right')
    upfld = 'R_upper_arm';
    lofld = 'R_lower_arm';
else
    upfld = 'L_upper_arm';
    lofld = 'L_lower_arm';
end

tslength = size(Q.(lofld).q_opt,2);

p_shL = [0;0;0];
if isfield(ref,upfld)
    p0_uL = par.la * ref.(upfld).g / norm(ref.(upfld).g);
end
p0_lL = par.lf * ref.(lofld).g / norm(ref.(lofld).g);


for i=1:tslength
    if isfield(Q,upfld)
        p_ul(:,i) = quatrot([-Q.(upfld).q_opt(1:3,i); Q.(upfld).q_opt(4,i)] ,p0_uL);
        p_ul(:,i) = quatrot([-Q.(upfld).q_ref(1:3); Q.(upfld).q_ref(4)],p_ul(1:3,i));
    else
        p_ul(:,i) = zeros(4,1);
    end
    p_ll1(:,i) = quatrot([-Q.(lofld).q_opt(1:3,i); Q.(lofld).q_opt(4,i)] ,p0_lL);
    p_ll1(:,i) = quatrot([-Q.(lofld).q_ref(1:3); Q.(lofld).q_ref(4)],p_ll1(1:3,i));
    p_ll(:,i) = p_ll1(:,i) + p_ul(:,i);%+ p_ul(:,i);
    n_p_ul = p_ul(:,i)/norm(p_ul(:,i));
    n_p_ll1 = p_ll1(:,i)/norm(p_ll1(:,i));
    ang.Lelbow(i) = acos(dot(n_p_ll1(1:3),n_p_ul(1:3)));
    rotax(:,i) = cross(n_p_ll1(1:3),n_p_ul(1:3));
    rotax(:,i) = rotax(:,i)/norm(rotax(:,i));
end

if ploten

close all
figure
hold on

    for i=1:tslength
        clf
        line([p_shL(1) p_ul(1,i)],[p_shL(2) p_ul(2,i)],[p_shL(3) p_ul(3,i)],'Color',[0 0 1],'Marker','*')
        line([p_ul(1,i) p_ll(1,i)],[p_ul(2,i) p_ll(2,i)],[p_ul(3,i) p_ll(3,i)],'Color',[1 0 0],'Marker','*')
        title('Stick arm-forearm')
        xlabel('x'), ylabel('y'), zlabel('z')
        xlim([-1000 1000]), ylim([-400 500]), zlim([-500 1000]),grid
            set(gca,'CameraPosition',300*[2 -1 1],'CameraTarget',[0 0 0])
%         set(gca,'CameraPosition',300*[0 0 1],'CameraTarget',[0 0 0])

        pause(0.01)
%         F(i)=getframe;

    end
%     movie(F)
%     movie2avi(F,'movie.avi','fps',100)

end

figure
hold on, grid
plot3(p_shL(1),p_shL(2),p_shL(3),'*k')
plot3(p_ul(1,:),p_ul(2,:),p_ul(3,:),'b')
plot3(p_ll(1,:),p_ll(2,:),p_ll(3,:),'r')
xlabel('x'), ylabel('y'), zlabel('z')
% set(gca,'CameraPosition',[-300 250 -100],'CameraTarget',[0 250 -100])


if camref~=0
    figure
    plot(time',ang.Lelbow*180/pi)
    hold on
    plot(camref.time,camref.ang.el*180/pi,'r*','MarkerSize',2)
end
pos.Lelbow=p_ul(1:3,:);
pos.Lwrist=p_ll(1:3,:);



