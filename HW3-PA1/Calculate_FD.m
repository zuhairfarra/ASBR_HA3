function FD = Calculate_FD(Cal_Body_Data,D_Data,N_D)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

d_data = Cal_Body_Data(2:N_D+1,:);

d_sum = zeros(1,3);
for idx = 1:N_D
    d_sum = d_sum + d_data(idx,:);
end
d_bar = d_sum/N_D;

d_mod = d_data;
for idx=1:N_D
    d_mod(idx,:) = d_data(idx,:) - d_bar;
end

D = D_Data;

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
for idx = 1:length(d_mod(:,1))
    a = d_mod(idx,:)';
    b = D_mod(idx,:)';
    
    M((idx-1)*4+1:idx*4,:) = [0 (b-a)';...
                  (b-a) ssm(b+a)];
end

[U_1,S_1,V_1] = svd(M)

Quat1 = quaternion(V_1(:,end)');
R1 = quat2rotm(Quat1);

p = D_bar' - R1*d_bar';

FD = [R1 p;...
     0 0 0 1];

end