%% Import Data
My_Data = pa1debugacalbody;

N_D = My_Data(1,1);
N_A = My_Data(1,2);
N_C = My_Data(1,3);

n = length(My_Data(:,1));

d_data = pa1debugacalbody(2:N_D+1,:);
a_data = pa1debugacalbody(N_D+2:N_D+N_A+1,:);
c_data = pa1debugacalbody(n-N_C:n,:);

%% Registration algorithm

d_sum = zeros(1,3);
for idx = 1:N_D
    d_sum = d_sum + d_data(idx,:);
end
d_bar = d_sum/N_D

d_mod = d_data;
for idx=1:N_D
    d_mod(idx,:) = d_data(idx,:) - d_bar;
end

%%

a_sum = zeros(1,3);
for idx = 1:N_A
    a_sum = a_sum + a_data(idx,:);
end
a_bar = a_sum/N_A

a_mod = a_data;
for idx=1:N_A
    a_mod(idx,:) = a_data(idx,:) - a_bar;
end

c_sum = zeros(1,3);
for idx = 1:N_C
    c_sum = c_sum + c_data(idx,:);
end
c_bar = c_sum/N_C

c_mod = c_data;
for idx=1:N_C
    c_mod(idx,:) = c_data(idx,:) - c_bar;
end

%%
Data_2 = pa1debugacalreadings;
N_D = Data_2(1,1);
N_A = Data_2(1,2);
N_C = Data_2(1,3);
N_frame = 8;

n = length(Data_2(:,1));
idx = 1;
S = N_D + N_A + N_C;

frame_1 = Data_2(2:S+1,:);
frame_2 = Data_2(S+2:2*S+1,:);
frame_3 = Data_2(2*S+2:3*S+1,:);
frame_4 = Data_2(3*S+2:4*S+1,:);
frame_5 = Data_2(4*S+2:5*S+1,:);
frame_6 = Data_2(5*S+2:6*S+1,:);
frame_7 = Data_2(6*S+2:7*S+1,:);
frame_8 = Data_2(7*S+2:8*S+1,:);

D_data(:,:,1) = frame_1(1:N_D,:);
D_data(:,:,2) = frame_2(1:N_D,:);

D = D_data(:,:,1);

D_sum = zeros(1,3);
for idx = 1:N_D
    D_sum = D_sum + D(idx,:);
end
D_bar = D_sum/N_D;

D_mod = D;
for idx=1:N_D
    D_mod(idx,:) = D(idx,:) - D_bar;
end

M = zeros(4*N_D,4);
for idx = 1:length(d_mod(1,:))
    a = d_mod(idx,:)';
    b = D_mod(idx,:)';
    
    M((idx-1)*4+1:idx*4,:) = [0 (b-a)';...
                  (b-a) ssm(b+a)];
end

[U_1,S_1,V_1] = svd(M);

Quat1 = quaternion(V_1(:,end)');
R1 = quat2rotm(Quat1);

p = D_bar' - R1*d_bar';

T = [R1 p;...
     0 0 0 1];
