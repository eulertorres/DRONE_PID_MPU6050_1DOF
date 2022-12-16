clc
clear all
close all

%% DATA EXTRACTING | phi = roll | theta = pitch | psi = yaw
% acc_x | acc_y | acc_z | g_x | g_y | g_z | time | 
%   1       2       3      4     5     6     7     
fileID = fopen('data.txt','r');
DATA_SI = fscanf(fileID,'%f', [7 Inf]);
N = size(DATA_SI);
Nsamples = N(2)-1; %length of DATA
N_fingers = 1;
fclose(fileID);

%Pre-allocation of loop variables
% PhiSaved = zeros(N_fingers, N(2));
% ThetaSaved = PhiSaved;
% PsiSaved   = PhiSaved;

unit_transform_gyro = pi/180;

%Time ms -> s   
DATA_SI(7,:) = (DATA_SI(7,:)-DATA_SI(7,1))/1000;

%Angles from 5 fingers aquired from dataset
% for z = 1:N_fingers
%     PhiSaved(z, :)   = DATA_SI((z-1)*20+13, :) ;   % pitch
%     ThetaSaved(z, :) = DATA_SI((z-1)*20+14, :) ;   % roll
%     PsiSaved(z, :)   = DATA_SI((z-1)*20+15, :) ;   % yaw
%     
%     %Gyro LSB -> deg/s -> rad/s 
%     DATA_SI((z-1)*20+4,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+4,:);
%     DATA_SI((z-1)*20+5,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+5,:);
%     DATA_SI((z-1)*20+6,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+6,:);
% end
  
    x = [0 0];
    y = [-1000,1000];
%% Plot the angle graph
%     figure()
% for z = 1:N_fingers
%     subplot(2,5,z)
%     P1=plot(DATA_SI(7,:), PhiSaved(z,:), 'r'); % roll
%     hold on
%     P2=plot(DATA_SI(7,:), ThetaSaved(z,:), 'b'); % pitch
%     P3=plot(DATA_SI(7,:), PsiSaved(z,:), 'g'); % yaw
%     refline([0 0])
%     title('IMU [graus]')
%     Timeline_1_1(z) = line('XData',x,'YData',y);
%     TimeValue_1_1(z)= xlabel('');
%     legend([P1 P2 P3],{'Phi', 'Theta', 'Psi'},'Location','northwest','AutoUpdate','off');
%     axis([0 DATA_SI(7,Nsamples) -180 180])
% end
% 

    %% Plot the acceleration graph
    figure()
for z=1:N_fingers
    %subplot(5,2,(z-1)*2+1)
    plot(DATA_SI(7,:),DATA_SI((z-1)*20+1,:),'r',DATA_SI(7,:),DATA_SI((z-1)*20+2,:),'g',DATA_SI(7,:),DATA_SI((z-1)*20+3,:),'b'); % ax ay az
    refline([0 0])
    title('Accel (m/s^2) IMU');
    Timeline_2_1(z) = line('XData',x,'YData',y);
    TimeValue_2_1(z)= xlabel('');
    legend({'AccX', 'AccY', 'AccZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(7,Nsamples) -2.1 2.1])
end
    %% Plot the angular velocity graph
%     figure()
% for z=1:N_fingers
%     subplot(5,2,(z-1)*2+1)
%     plot(DATA_SI(7,:),DATA_SI((z-1)*20+4,:),'r',DATA_SI(7,:),DATA_SI((z-1)*20+5,:),'g',DATA_SI(7,:),DATA_SI((z-1)*20+6,:),'b'); % gx gy gz
%     refline([0 0])
%     title('Gyro (rad/s) IMU')
%     Timeline_3_1(z) = line('XData',x,'YData',y);
%     TimeValue_3_1(z)= xlabel('');
%     %legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
%     axis([0 DATA_SI(7,Nsamples) -3 3])
% end

    
    %% Graphical Plot & Dynamic Plot
    %set IMU Object's vertex
%     posx = 1.5;
%     posy = 3;
%     posz = 0.5;
%     
%     % Poisition in y axis for the IMU 1 of each finger
%     %       Finger: 0 |  1  |  3 |  4 | 5
%     difference_y = [-9, 5, 10, 8, 4];
%     % Poisition in x axis for the IMU 1 of each finger
%     difference_x = [-18, -8, 0, 8, 16];
%     imu_distance = -15;
%     axis_size = 30;
%     
%     fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
%     
%     %Plot Object
%     figure()
%     view(3)
%     axis([-axis_size axis_size -axis_size axis_size -axis_size axis_size])
%     title('Finger state')
%     Realtime = xlabel(' ');
%     
%     Cuboid = zeros(8,3,5);
%     Cuboid2 = zeros(8,3,5);
%     
% for z = 1:N_fingers
%     Cuboid_1(z,:)=[-posx+difference_x(z),-posy-difference_y(z),-posz];
%     Cuboid_2(z,:)=[posx+difference_x(z),-posy-difference_y(z),-posz];
%     Cuboid_3(z,:)=[posx+difference_x(z),posy-difference_y(z),-posz];
%     Cuboid_4(z,:)=[-posx+difference_x(z),posy-difference_y(z),-posz];
%     Cuboid_5(z,:)=[-posx+difference_x(z),-posy-difference_y(z),posz];
%     Cuboid_6(z,:)=[posx+difference_x(z),-posy-difference_y(z),posz];
%     Cuboid_7(z,:)=[posx+difference_x(z),posy-difference_y(z),posz];
%     Cuboid_8(z,:)=[-posx+difference_x(z),posy-difference_y(z),posz];
% 
%     Cuboid(:,:,z)=[Cuboid_1(z,:); Cuboid_2(z,:); Cuboid_3(z,:); Cuboid_4(z,:); Cuboid_5(z,:); Cuboid_6(z,:); Cuboid_7(z,:); Cuboid_8(z,:)];
% 
%     Obejct_IMU(z) = patch('Faces', fac, 'Vertices', [0, 0, 0], 'FaceVertexCData',(1:6)','FaceColor','flat');
% end
%     % ?

%Animate
for k=1:Nsamples-1
    
    dt=(DATA_SI(7,k+1)-DATA_SI(7,k));
    %set(Realtime, 'String', sprintf('time = %.2f [s]', DATA_SI(7, k)))
%     
     for z=1:N_fingers
%         %Rotation matrix for vertex
%         Rz = [cosd(PsiSaved(z,k)) -sind(PsiSaved(z,k)) 0; sind(PsiSaved(z,k)) cosd(PsiSaved(z,k)) 0; 0 0 1];
%         Ry = [cosd(ThetaSaved(z,k)) 0 sind(ThetaSaved(z,k)); 0 1 0; -sind(ThetaSaved(z,k)) 0 cosd(ThetaSaved(z,k))];
%         Rx = [1 0 0; 0 -cosd(PhiSaved(z,k)) sind(PhiSaved(z,k)); 0 -sind(PhiSaved(z,k)) -cosd(PhiSaved(z,k))];
% 
%         for j=1:8
%             %Rotated vertex
%             Result_1(j,:,k,z) = Rx*Ry*Rz*Cuboid(j,:, z)';
%         end
% 
%         %Display realtime object and time
%         set(Obejct_IMU(z), 'Vertices', Result_1(:,:,k,z))

        %Display realtime sensor values
%          set(TimeValue_1_1(z), 'String', sprintf('Phi: %.2f  Theta: %.2f  Psi: %.2f', PhiSaved(z,k), ThetaSaved(z,k), PsiSaved(z,k)));
         set(TimeValue_2_1(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+1,k), DATA_SI((z-1)*20+2,k), DATA_SI((z-1)*20+3,k)));	%acc1
%          set(TimeValue_3_1(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+4,k), DATA_SI((z-1)*20+5,k), DATA_SI((z-1)*20+6,k)));	%gyro1
        %Display timeline
%          set(Timeline_1_1(z), 'XData', [DATA_SI(7, k) DATA_SI(7, k)]);
         set(Timeline_2_1(z), 'XData', [DATA_SI(7, k) DATA_SI(7, k)]);
%          set(Timeline_3_1(z), 'XData', [DATA_SI(7, k) DATA_SI(7, k)]);
    end
    % pause(dt)
    drawnow
end