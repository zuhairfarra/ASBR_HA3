[q1,q2,t1,t2] = data_quaternion();

T = Hand_in_Eye_Calibration(q1,q2,t1,t2);
%%
idx = 1;
jdx = 2;

E1 = [quat2rotm(q1(idx,:)) t1(idx,:)';...
          0 0 0 1];
    
E2 = [quat2rotm(q1(jdx,:)) t1(jdx,:)';...
      0 0 0 1];

S1 = [quat2rotm(q2(idx,:)) t2(idx,:)';...
      0 0 0 1];

S2 = [quat2rotm(q2(jdx,:)) t2(jdx,:)';...
      0 0 0 1];

LS = E1*T*S1
RS = E2*T*S2