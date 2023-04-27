% ellie andreyka 
% ME 397: ASBR THA3
% PA 1: GOAL 2, 4, & 5

clear
clc 
close all 

%% Goal 2 and 4
filenames = {'pa1-debug-a-empivot.txt';
             'pa1-debug-b-empivot.txt';
             'pa1-debug-c-empivot.txt';
             'pa1-debug-d-empivot.txt';
             'pa1-debug-e-empivot.txt';
             'pa1-debug-f-empivot.txt';
             'pa1-debug-g-empivot.txt';
             'pa1-unknown-h-empivot.txt';
             'pa1-unknown-i-empivot.txt';
             'pa1-unknown-j-empivot.txt';
             'pa1-unknown-k-empivot.txt'};

casenum = struct;
for i = 1: length(filenames)
    filename = string(filenames(i));
    Gj_now = get_data_array(filename);
    for j = 1:12
        % get Gj for each frame 
        Gj = Gj_now(6*(j-1)+1:j*6,:);
        casenum(i).frame(j).Gj = Gj; 

        % midpoint of observed points
        NG = length(Gj);
        G0 = 1/NG*(sum(Gj));
        
        % translate all points relative to midpoint 
        gj = Gj-G0;
        gj_avg = 1/NG*(sum(gj));

        % get transformation FG(k) for each "frame" k
        M = zeros(4*NG,4);
        for idx = 1:length(gj(:,1))
            a = gj(idx,:)';
            b = Gj(idx,:)';
            M((idx-1)*4+1:idx*4,:) = [0 (b-a)'; (b-a) ssm(b+a)];
        end
        [U_1,S_1,V_1] = svd(M);
        Quat1 = quaternion(V_1(:,end)');
        R1 = quat2rotm(Quat1);
        p = gj_avg' - R1*G0';
        FG = [R1 p; 0 0 0 1];
        casenum(i).frame(j).FG = FG;
        
        % get tg
        % Ax = b --> x = lsqr(A,b)
        b = lsqr([R1,-eye(3,3)],-p);
        casenum(i).frame(j).tg = b;
    end 
end 


%% Goal 5
% Apply the optical tracking data to perform a pivot calibration of the optical
% tracking probe. The suggested method is the same as above except that you should first
% use your value for FD to transform the optical tracker beacon positions into EM tracker
% coordinates. Note that the optical tracker may not be in exactly the same position and
% orientation with respect to the EM tracker base for each observation frame of optical
% tracker data, so this is an important step. 

% Hi in pivot 
% use FD to transform Hi (optical tracker beacon positions) --> Gi (EM tracker coordinates)

filenames_cal_body = {'pa1-debug-a-calbody.txt';
             'pa1-debug-b-calbody.txt';
             'pa1-debug-c-calbody.txt';
             'pa1-debug-d-calbody.txt';
             'pa1-debug-e-calbody.txt';
             'pa1-debug-f-calbody.txt';
             'pa1-debug-g-calbody.txt';
             'pa1-unknown-h-calbody.txt';
             'pa1-unknown-i-calbody.txt';
             'pa1-unknown-j-calbody.txt';
             'pa1-unknown-k-calbody.txt'};

filenames_D_H= {'pa1-debug-a-optpivot.txt';
             'pa1-debug-b-optpivot.txt';
             'pa1-debug-c-optpivot.txt';
             'pa1-debug-d-optpivot.txt';
             'pa1-debug-e-optpivot.txt';
             'pa1-debug-f-optpivot.txt';
             'pa1-debug-g-optpivot.txt';
             'pa1-unknown-h-optpivot.txt';
             'pa1-unknown-i-optpivot.txt';
             'pa1-unknown-j-optpivot.txt';
             'pa1-unknown-k-optpivot.txt'};

casenum_pt2 = struct;
for i = 1: length(filenames_D_H)
    filename_Hj = string(filenames_D_H(i));
    D_H = get_data_array(filename_Hj);
    filename_calB = string(filenames_cal_body(i));
    Cal_body_data = get_data_array(filename_calB);
    for j = 1:12
        % get D and H for each frame 
        D_frame = D_H(8*(j-1)+1+(j-1)*6:j*8+(j-1)*6,:);
        H_frame = D_H(j*8+6*(j-1)+1:j*8+j*6,:);
        casenum_pt2(i).frame(j).D_frame = D_frame; 
        casenum_pt2(i).frame(j).H_frame = H_frame; 

        % get FD and Gj
        FD_frame = Calculate_FD(Cal_body_data,D_frame,8);
        
        Gj = zeros(size(H_frame));
        % get Gj 
        for m = 1:6
            FD_frame(1:3,1:3)
            H_frame(m,:)
            Gj(m,:) = FD_frame(1:3,1:3)*H_frame(m,:)'+FD_frame(1:3,4);
        end 
    
        % midpoint of observed points
        NG = length(Gj);
        G0 = 1/NG*(sum(Gj));
        
        % translate all points relative to midpoint 
        gj = Gj-G0;
        gj_avg = 1/NG*(sum(gj));

        % get transformation FG(k) for each "frame" k
        M = zeros(4*NG,4);
        for idx = 1:length(gj(:,1))
            a = gj(idx,:)';
            b = Gj(idx,:)';
            M((idx-1)*4+1:idx*4,:) = [0 (b-a)'; (b-a) ssm(b+a)];
        end
        [U_1,S_1,V_1] = svd(M);
        Quat1 = quaternion(V_1(:,end)');
        R1 = quat2rotm(Quat1);
        p = gj_avg' - R1*G0';
        FG = [R1 p; 0 0 0 1];
        casenum_pt2(i).frame(j).FG = FG;
        
        % get tg
        % Ax = b --> x = lsqr(A,b)
        b = lsqr([R1,-eye(3,3)],-p);
        casenum_pt2(i).frame(j).tg = b;
    end 
end 

%% deleted 
% Ax = b
% P_dimple = FG(i)*tG
% for i = 1: length(Gj)
%     FG(i) = Gj(i)/gj(i)
%     % Ax = b
%     % P_dimple = FG(i)*tG
%     tG = lsqr(P_dimple,FG(i));
% end 
% create a struct where 
% 11 cases --> 12 frames in each case 
% names = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k'];
% filename = 'pa1-debug-a-empivot.txt';
% % Gj_case{i} = get_Gj(filename)
% %% midpoint of observed points
% NG = length(Gj);
% G0 = 1/NG*(sum(Gj));
% 
% % translate all points relative to midpoint 
% gj = Gj-G0;
% gj_avg = 1/NG*(sum(gj));
% % get transformation FG(k) for each "frame" k
% M = zeros(4*NG,4);
% for idx = 1:length(gj(:,1))
%     a = gj(idx,:)';
%     b = Gj(idx,:)';
%     M((idx-1)*4+1:idx*4,:) = [0 (b-a)'; (b-a) ssm(b+a)];
% end
% [U_1,S_1,V_1] = svd(M);
% Quat1 = quaternion(V_1(:,end)');
% R1 = quat2rotm(Quat1);
% p = gj_avg' - R1*G0';
% FG = [R1 p; 0 0 0 1];
% % Ax = b --> x = lsqr(A,b)
% b = lsqr([R1,-eye(3,3)],-p)


