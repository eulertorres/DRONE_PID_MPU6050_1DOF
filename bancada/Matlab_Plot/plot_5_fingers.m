clc
clear all
close all

%% DATA EXTRACTING | phi = roll | theta = pitch | psi = yaw
% acc_x_1 | acc_y_1 | acc_z_1 | g_x_1 | g_y_1 | g_z_1 | acc_x_2 | acc_y_2 | % acc_z_2 | g_x_2 | g_y_2 | g_z_2 | pitch_1 | roll_1 | yaw_1 | pitch_2 | roll_2 | yaw_2 | pressure | position | closed | time | 
%   1           2       3         4       5        6      7         8           9         10     11       12      13       14       15        16       17       18      19          20        21      102     
fileID = fopen('data.txt','r');
DATA_SI = fscanf(fileID,'%f', [102 Inf]);
N = size(DATA_SI);
Nsamples = N(2)-1; %length of DATA
N_fingers = 5;
fclose(fileID);

%Pre-allocation of loop variables
PhiSaved = zeros(5, N(2));
ThetaSaved = PhiSaved;
PsiSaved   = PhiSaved;
PhiSaved2 = PhiSaved;
ThetaSaved2 = PhiSaved;
PsiSaved2   = PhiSaved;

unit_transform_gyro = pi/180;

%Time ms -> s   
DATA_SI(102,:) = (DATA_SI(102,:)-DATA_SI(102,1))/1000;

%Angles from 5 fingers aquired from dataset
for z = 1:N_fingers
    PhiSaved(z, :)   = DATA_SI((z-1)*20+13, :) ;   % pitch
    ThetaSaved(z, :) = DATA_SI((z-1)*20+14, :) ;   % roll
    PsiSaved(z, :)   = DATA_SI((z-1)*20+15, :) ;   % yaw

    PhiSaved2(z, :)   = DATA_SI((z-1)*20+16, :) ;   % pitch
    ThetaSaved2(z, :) = DATA_SI((z-1)*20+17, :) ;   % roll
    PsiSaved2(z, :)   = DATA_SI((z-1)*20+18, :) ;   % yaw
    
    %Gyro LSB -> deg/s -> rad/s 
    DATA_SI((z-1)*20+4,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+4,:);
    DATA_SI((z-1)*20+5,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+5,:);
    DATA_SI((z-1)*20+6,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+6,:);
    
    DATA_SI((z-1)*20+10,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+10,:);
    DATA_SI((z-1)*20+11,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+11,:);
    DATA_SI((z-1)*20+12,:) = (unit_transform_gyro)*DATA_SI((z-1)*20+12,:);
end
  
    x = [0 0];
    y = [-1000,1000];
%% Plot the angle graph
    figure()
for z = 1:N_fingers
    subplot(2,5,z)
    P1=plot(DATA_SI(102,:), PhiSaved(z,:), 'r'); % roll
    hold on
    P2=plot(DATA_SI(102,:), ThetaSaved(z,:), 'b'); % pitch
    P3=plot(DATA_SI(102,:), PsiSaved(z,:), 'g'); % yaw
    refline([0 0])
    title('IMU 1 [graus]')
    Timeline_1_1(z) = line('XData',x,'YData',y);
    TimeValue_1_1(z)= xlabel('');
    %legend([P1 P2 P3],{'Phi', 'Theta', 'Psi'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -180 180])
    
    subplot(2,5,z+5)
    P1=plot(DATA_SI(102,:), PhiSaved2(z,:), 'r'); % roll
    hold on
    P2=plot(DATA_SI(102,:), ThetaSaved2(z,:), 'b'); % pitch
    P3=plot(DATA_SI(102,:), PsiSaved2(z,:), 'g'); % yaw
    refline([0 0])
    title('IMU 2 [graus]')
    Timeline_1_2(z) = line('XData',x,'YData',y);
    TimeValue_1_2(z) = xlabel('');
    %legend([P1 P2 P3],{'Phi', 'Theta', 'Psi'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -180 180])
end


    %% Plot the acceleration graph
    figure()
for z=1:N_fingers
    subplot(5,2,(z-1)*2+1)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+1,:),'r',DATA_SI(102,:),DATA_SI((z-1)*20+2,:),'g',DATA_SI(102,:),DATA_SI((z-1)*20+3,:),'b'); % ax ay az
    refline([0 0])
    title('Accel (m/s^2) MPU 1');
    Timeline_2_1(z) = line('XData',x,'YData',y);
    TimeValue_2_1(z)= xlabel('');
    %legend({'AccX', 'AccY', 'AccZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -2.1 2.1])
    
    subplot(5,2,(z-1)*2+2)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+7,:),'r',DATA_SI(102,:),DATA_SI((z-1)*20+8,:),'g',DATA_SI(102,:),DATA_SI((z-1)*20+9,:),'b'); % ax ay az
    refline([0 0])
    title('Accel (m/s^2) MPU 2');
    Timeline_2_2(z) = line('XData',x,'YData',y);
    TimeValue_2_2(z)= xlabel('');
    %legend({'AccX', 'AccY', 'AccZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -2.1 2.1])
end
    %% Plot the angular velocity graph
    figure()
for z=1:N_fingers
    subplot(5,2,(z-1)*2+1)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+4,:),'r',DATA_SI(102,:),DATA_SI((z-1)*20+5,:),'g',DATA_SI(102,:),DATA_SI((z-1)*20+6,:),'b'); % gx gy gz
    refline([0 0])
    title('Gyro (rad/s) MPU 1')
    Timeline_3_1(z) = line('XData',x,'YData',y);
    TimeValue_3_1(z)= xlabel('');
    %legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -3 3])
    
    subplot(5,2,(z-1)*2+2)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+10,:),'r',DATA_SI(102,:),DATA_SI((z-1)*20+11,:),'g',DATA_SI(102,:),DATA_SI((z-1)*20+12,:),'b'); % gx gy gz
    refline([0 0])
    title('Gyro (rad/s) MPU 2')
    Timeline_3_2(z) = line('XData',x,'YData',y);
    TimeValue_3_2(z)= xlabel('');
    %legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -3 3])
end
    %% Plot the magnetic flux graph
    % subplot(1,3,3)
    % plot(DATA_SI(13,:),DATA_SI(7,:),'r',DATA_SI(13,:),DATA_SI(8,:),'g',DATA_SI(13,:),DATA_SI(9,:),'b');
    % refline([0 0])
    % title('Magnetic flux density (uT)')
    % Timeline_4 = line('XData',x,'YData',y);
    % TimeValue_4= xlabel('');
    % legend({'MagX', 'MagY', 'MagZ'},'Location','northwest','AutoUpdate','off');
    % axis([0 DATA_SI(13,Nsamples) -40 40])

    %% Plot the location [mm] and magnitude[N] of the grip of each finger
    figure()
for z=1:N_fingers
    subplot(5,2,(z-1)*2+1)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+19)); % pressure
    refline([0 0])
    title('Pressure [N] MPU 1')
    %Timeline_3_1(z) = line('XData',x,'YData',y);
    %TimeValue_3_1(z)= xlabel('');
    %legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -1000 1000])
    
    subplot(5,2,(z-1)*2+2)
    plot(DATA_SI(102,:),DATA_SI((z-1)*20+20,:)); % Position
    refline([0 0])
    title('Position [N] MPU 2')
    %Timeline_3_2(z) = line('XData',x,'YData',y);
    %TimeValue_3_2(z)= xlabel('');
    %legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
    axis([0 DATA_SI(102,Nsamples) -1 100])
end
    
    %% Graphical Plot & Dynamic Plot
    %set IMU Object's vertex
    posx = 1.5;
    posy = 3;
    posz = 0.5;
    
    % Poisition in y axis for the IMU 1 of each finger
    %       Finger: 0 |  1  |  3 |  4 | 5
    difference_y = [-9, 5, 10, 8, 4];
    % Poisition in x axis for the IMU 1 of each finger
    difference_x = [-18, -8, 0, 8, 16];
    imu_distance = -15;
    axis_size = 30;
    
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    
    %Plot Object
    figure()
    view(3)
    axis([-axis_size axis_size -axis_size axis_size -axis_size axis_size])
    title('Finger state')
    Realtime = xlabel(' ');
    
    Cuboid = zeros(8,3,5);
    Cuboid2 = zeros(8,3,5);
    
for z = 1:N_fingers
    Cuboid_1(z,:)=[-posx+difference_x(z),-posy-difference_y(z),-posz];
    Cuboid_2(z,:)=[posx+difference_x(z),-posy-difference_y(z),-posz];
    Cuboid_3(z,:)=[posx+difference_x(z),posy-difference_y(z),-posz];
    Cuboid_4(z,:)=[-posx+difference_x(z),posy-difference_y(z),-posz];
    Cuboid_5(z,:)=[-posx+difference_x(z),-posy-difference_y(z),posz];
    Cuboid_6(z,:)=[posx+difference_x(z),-posy-difference_y(z),posz];
    Cuboid_7(z,:)=[posx+difference_x(z),posy-difference_y(z),posz];
    Cuboid_8(z,:)=[-posx+difference_x(z),posy-difference_y(z),posz];

    Cuboid_1_2(z,:)=[-posx+difference_x(z),-posy-difference_y(z)+imu_distance,-posz];
    Cuboid_2_2(z,:)=[posx+difference_x(z),-posy-difference_y(z)+imu_distance,-posz];
    Cuboid_3_2(z,:)=[posx+difference_x(z),posy-difference_y(z)+imu_distance,-posz];
    Cuboid_4_2(z,:)=[-posx+difference_x(z),posy-difference_y(z)+imu_distance,-posz];
    Cuboid_5_2(z,:)=[-posx+difference_x(z),-posy-difference_y(z)+imu_distance,posz];
    Cuboid_6_2(z,:)=[posx+difference_x(z),-posy-difference_y(z)+imu_distance,posz];
    Cuboid_7_2(z,:)=[posx+difference_x(z),posy-difference_y(z)+imu_distance,posz];
    Cuboid_8_2(z,:)=[-posx+difference_x(z),posy-difference_y(z)+imu_distance,posz];

    Cuboid(:,:,z)=[Cuboid_1(z,:); Cuboid_2(z,:); Cuboid_3(z,:); Cuboid_4(z,:); Cuboid_5(z,:); Cuboid_6(z,:); Cuboid_7(z,:); Cuboid_8(z,:)];
    Cuboid2(:,:,z)=[Cuboid_1_2(z,:); Cuboid_2_2(z,:); Cuboid_3_2(z,:); Cuboid_4_2(z,:); Cuboid_5_2(z,:); Cuboid_6_2(z,:); Cuboid_7_2(z,:); Cuboid_8_2(z,:)];

    Obejct_IMU(z) = patch('Faces', fac, 'Vertices', [0, 0, 0], 'FaceVertexCData',(1:6)','FaceColor','flat');
    Obejct_IMU2(z) = patch('Faces', fac, 'Vertices', [0, 0, 0], 'FaceVertexCData',(1:6)','FaceColor','flat');
end
    % ?

%Animate
for k=1:Nsamples-1
    
    dt=(DATA_SI(102,k+1)-DATA_SI(102,k));
    set(Realtime, 'String', sprintf('time = %.2f [s]', DATA_SI(102, k)))
    
    for z=1:N_fingers
        %Rotation matrix for vertex
        Rz = [cosd(PsiSaved(z,k)) -sind(PsiSaved(z,k)) 0; sind(PsiSaved(z,k)) cosd(PsiSaved(z,k)) 0; 0 0 1];
        Ry = [cosd(ThetaSaved(z,k)) 0 sind(ThetaSaved(z,k)); 0 1 0; -sind(ThetaSaved(z,k)) 0 cosd(ThetaSaved(z,k))];
        Rx = [1 0 0; 0 -cosd(PhiSaved(z,k)) sind(PhiSaved(z,k)); 0 -sind(PhiSaved(z,k)) -cosd(PhiSaved(z,k))];

        Rz2 = [cosd(PsiSaved2(z,k)) -sind(PsiSaved2(z,k)) 0; sind(PsiSaved2(z,k)) cosd(PsiSaved2(z,k)) 0; 0 0 1];
        Ry2 = [cosd(ThetaSaved2(z,k)) 0 sind(ThetaSaved2(z,k)); 0 1 0; -sind(ThetaSaved2(z,k)) 0 cosd(ThetaSaved2(z,k))];
        Rx2 = [1 0 0; 0 -cosd(PhiSaved2(z,k)) sind(PhiSaved2(z,k)); 0 -sind(PhiSaved2(z,k)) -cosd(PhiSaved2(z,k))];

        for j=1:8
            %Rotated vertex
            Result_1(j,:,k,z) = Rx*Ry*Rz*Cuboid(j,:, z)';
            Result_2(j,:,k,z) = Rx2*Ry2*Rz2*Cuboid2(j,:, z)';
        end

        %Display realtime object and time
        set(Obejct_IMU(z), 'Vertices', Result_1(:,:,k,z))
        set(Obejct_IMU2(z), 'Vertices', Result_2(:,:,k,z))

        %Display realtime sensor values
         set(TimeValue_1_1(z), 'String', sprintf('Phi: %.2f  Theta: %.2f  Psi: %.2f', PhiSaved(z,k), ThetaSaved(z,k), PsiSaved(z,k)));
         set(TimeValue_1_2(z), 'String', sprintf('Phi: %.2f  Theta: %.2f  Psi: %.2f', PhiSaved2(z,k), ThetaSaved2(z,k), PsiSaved2(z,k)));
         set(TimeValue_2_1(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+1,k), DATA_SI((z-1)*20+2,k), DATA_SI((z-1)*20+3,k)));	%acc1
         set(TimeValue_2_2(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+7,k), DATA_SI((z-1)*20+8,k), DATA_SI((z-1)*20+9,k)));	%acc2
         set(TimeValue_3_1(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+4,k), DATA_SI((z-1)*20+5,k), DATA_SI((z-1)*20+6,k)));	%gyro1
         set(TimeValue_3_2(z), 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI((z-1)*20+10,k), DATA_SI((z-1)*20+11,k), DATA_SI((z-1)*20+12,k))); %gyro2
        %set(TimeValue_4, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(7,k), DATA_SI(8,k), DATA_SI(9,k)));
        %Display timeline
         set(Timeline_1_1(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
         set(Timeline_1_2(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
         set(Timeline_2_1(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
         set(Timeline_2_2(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
         set(Timeline_3_1(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
         set(Timeline_3_2(z), 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
    end
    % pause(dt)
    drawnow
end