function F_X = Hand_in_Eye_Calibration(q1,q2,t1,t2)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
config_num = 9;
Mq = zeros(4*config_num,4);

for idx = 1:config_num
    
    if isequal(idx,10)
        jdx = 1;
    else
        jdx = idx+1;
    end
    
    E1 = [quat2rotm(q1(idx,:)) t1(idx,:)';...
          0 0 0 1];
    
    E2 = [quat2rotm(q1(jdx,:)) t1(jdx,:)';...
          0 0 0 1];
    
    S1 = [quat2rotm(q2(idx,:)) t2(idx,:)';...
          0 0 0 1];
    
    S2 = [quat2rotm(q2(jdx,:)) t2(jdx,:)';...
          0 0 0 1];
    
    A = E1\E2;
    B = S1*S2^(-1);
    
    R_A = A(1:3,1:3); p_A = A(1:3,4);
    R_B = B(1:3,1:3); p_B = B(1:3,4);
    
    q_A = rotm2quat(R_A);
    q_B = rotm2quat(R_B);
    
    s_A = q_A(1);
    s_B = q_B(1);
    v_A = q_A(2:4)';
    v_B = q_B(2:4)';
    
    Mq((idx-1)*4+1:idx*4,:) = [(s_A-s_B) -(v_A-v_B)';...
                               (v_A-v_B) (s_A-s_B)*eye(3,3)+ssm(v_A+v_B)];

    R_s((idx-1)*3+1:idx*3,:) = R_A-eye(3,3);
    p_As(:,idx) = p_A;
    p_Bs(:,idx) = p_B;

end

[U_Q,S_Q,V_Q] = svd(Mq);

Quat_Q = quaternion(V_Q(:,end)');
R_Q = quat2rotm(Quat_Q);

A_x = R_s;
for idx=1:config_num
    b_x((idx-1)*3+1:idx*3,1) = R_Q*p_Bs(:,idx)-p_A(:,idx);
end

x = lsqr(A_x,b_x);

F_X = [R_Q x;...
       0 0 0 1];

end