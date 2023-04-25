%% Problem 3
file_list = struct();
file_list.CalBody_FileList = {'pa1-debug-a-calbody.txt',...
                              'pa1-debug-b-calbody.txt',...
                              'pa1-debug-c-calbody.txt',...
                              'pa1-debug-d-calbody.txt',...
                              'pa1-debug-e-calbody.txt',...
                              'pa1-debug-f-calbody.txt',...
                              'pa1-debug-g-calbody.txt',...
                              'pa1-unknown-h-calbody.txt',...
                              'pa1-unknown-i-calbody.txt',...
                              'pa1-unknown-j-calbody.txt',...
                              'pa1-unknown-k-calbody.txt'};

file_list.CalReadings_FileList = {'pa1-debug-a-calreadings.txt',...
                                  'pa1-debug-b-calreadings.txt',...
                                  'pa1-debug-c-calreadings.txt',...
                                  'pa1-debug-d-calreadings.txt',...
                                  'pa1-debug-e-calreadings.txt',...
                                  'pa1-debug-f-calreadings.txt',...
                                  'pa1-debug-g-calreadings.txt',...
                                  'pa1-unknown-h-calreadings.txt',...
                                  'pa1-unknown-i-calreadings.txt',...
                                  'pa1-unknown-j-calreadings.txt',...
                                  'pa1-unknown-k-calreadings.txt'};

file_list.Output_FileList = {'pa1-debug-a-output1.txt',...
                             'pa1-debug-b-output1.txt',...
                             'pa1-debug-c-output1.txt',...
                             'pa1-debug-d-output1.txt',...
                             'pa1-debug-e-output1.txt',...
                             'pa1-debug-f-output1.txt',...
                             'pa1-debug-g-output1.txt',...
                             'pa1-unknown-h-output1.txt',...
                             'pa1-unknown-i-output1.txt',...
                             'pa1-unknown-j-output1.txt',...
                             'pa1-unknown-k-output1.txt'};

file_list.TEST_Output_FileList = {'TEST-pa1-debug-a-output1.txt',...
                             'TEST-pa1-debug-b-output1.txt',...
                             'TEST-pa1-debug-c-output1.txt',...
                             'TEST-pa1-debug-d-output1.txt',...
                             'TEST-pa1-debug-e-output1.txt',...
                             'TEST-pa1-debug-f-output1.txt',...
                             'TEST-pa1-debug-g-output1.txt',...
                             'TEST-pa1-unknown-h-output1.txt',...
                             'TEST-pa1-unknown-i-output1.txt',...
                             'TEST-pa1-unknown-j-output1.txt',...
                             'TEST-pa1-unknown-k-output1.txt'};

N_Files = length(file_list.CalBody_FileList);

%% Import Data
% My_Data = pa1debugacalbody;
% 
% N_D = My_Data(1,1);
% N_A = My_Data(1,2);
% N_c = My_Data(1,3);
% 
% n = length(My_Data(:,1));
% 
% d_data = pa1debugacalbody(2:N_D+1,:);
% a_data = pa1debugacalbody(N_D+2:N_D+N_A+1,:);
% c_data = pa1debugacalbody(n-N_C+1:n,:);
% 
% %% Registration algorithm
% 
d_sum = zeros(1,3);
for idx = 1:N_D
    d_sum = d_sum + d_data(idx,:);
end
d_bar = d_sum/N_D;

d_mod = d_data;
for idx=1:N_D
    d_mod(idx,:) = d_data(idx,:) - d_bar;
end
% 
% %%
% 
% a_sum = zeros(1,3);
% for idx = 1:N_A
%     a_sum = a_sum + a_data(idx,:);
% end
% a_bar = a_sum/N_A;
% 
% a_mod = a_data;
% for idx=1:N_A
%     a_mod(idx,:) = a_data(idx,:) - a_bar;
% end
% 
% c_sum = zeros(1,3);
% for idx = 1:N_C
%     c_sum = c_sum + c_data(idx,:);
% end
% c_bar = c_sum/N_C;
% 
% c_mod = c_data;
% for idx=1:N_C
%     c_mod(idx,:) = c_data(idx,:) - c_bar;
% end

%%
for kdx=1:N_Files

    My_Data = Import_data_from_file(file_list.CalBody_FileList{kdx});

    N_D = My_Data(1,1);
    N_A = My_Data(1,2);
    N_c = My_Data(1,3);
    
    n = length(My_Data(:,1));
    
    d_data = My_Data(2:N_D+1,:);
    a_data = My_Data(N_D+2:N_D+N_A+1,:);
    c_data = My_Data(n-N_c+1:n,:);

    a_sum = zeros(1,3);
    for idx = 1:N_A
        a_sum = a_sum + a_data(idx,:);
    end
    a_bar = a_sum/N_A;
    
    a_mod = a_data;
    for idx=1:N_A
        a_mod(idx,:) = a_data(idx,:) - a_bar;
    end
    
    c_sum = zeros(1,3);
    for idx = 1:N_c
        c_sum = c_sum + c_data(idx,:);
    end
    c_bar = c_sum/N_c;
    
    c_mod = c_data;
    for idx=1:N_c
        c_mod(idx,:) = c_data(idx,:) - c_bar;
    end

    Data_from_File = Import_data_from_file(file_list.CalReadings_FileList{kdx});
    Data_2 = Data_from_File;
    N_D = Data_2(1,1);
    N_A = Data_2(1,2);
    N_C = Data_2(1,3);
    N_frame = 8;
    
    S = N_D + N_A + N_C;
    
    for idx = 1:N_frame
        frames(:,:,idx) = Data_2((idx-1)*S+2:idx*S+1,:);
    end

    D_data(:,:,:) = frames(1:N_D,:,:);
    A_data(:,:,:) = frames(N_D+1:N_D+N_A,:,:);
    C_data(:,:,:) = frames(N_D+N_A+1:S,:,:);
    
    F_D = zeros(4,4,N_frame);
    F_A = zeros(4,4,N_frame);
    C_Expected = zeros(3,N_c,N_frame);
    
    for idx = 1:N_frame
        F_D(:,:,idx) = Calculate_FD(Data_2,D_data(:,:,idx),N_D);
    
        A = A_data(:,:,idx);
    
        A_sum = zeros(1,3);
        for jdx = 1:N_A
            A_sum = A_sum + A(jdx,:);
        end
        A_bar = A_sum/N_A;
        
        A_mod = A;
        for jdx=1:N_A
            A_mod(jdx,:) = A(jdx,:) - A_bar;
        end
        
        M = zeros(4*N_A,4);
        for jdx = 1:length(a_mod(:,1))
            a = a_mod(jdx,:)';
            b = A_mod(jdx,:)';
            
            M((jdx-1)*4+1:jdx*4,:) = [0 (b-a)';...
                          (b-a) ssm(b+a)];
        end
        
        [U_A,S_A,V_A] = svd(M);
        
        Quat_A = quaternion(V_A(:,end)');
        R_A = quat2rotm(Quat_A);
        
        p = A_bar' - R_A*a_bar';
        
        F_A(:,:,idx) = [R_A p;...
                      0 0 0 1];
    
        F_DA = F_D(:,:,idx)\F_A(:,:,idx);
        for jdx =1:N_c
            C_Expected(:,jdx,idx) = F_DA(1:3,4) + F_DA(1:3,1:3)*c_data(jdx,:)';
        end
        C_Output = C_Expected(:,:,idx)';
        
        if isequal(idx,1)
            fid = fopen(file_list.TEST_Output_FileList{kdx},"w");
        end

        formatSpec = "%6.2f, %6.2f, %6.2f\n";
        fprintf(fid,formatSpec,C_Output);
        
        if isequal(idx,N_frame)
            fclose(fid);
        end

    end

end
%% DEbug
My_Data = Import_data_from_file(file_list.CalBody_FileList{1});

N_D = My_Data(1,1);
d_data = My_Data(2:N_D+1,:);

d_sum = zeros(1,3);
for idx = 1:N_D
    d_sum = d_sum + d_data(idx,:);
end
d_bar = d_sum/N_D;

d_mod = d_data;
for idx=1:N_D
    d_mod(idx,:) = d_data(idx,:) - d_bar;
end

Data_2 = Import_data_from_file(file_list.CalReadings_FileList{1});
N_D = Data_2(1,1);
N_A = Data_2(1,2);
N_C = Data_2(1,3);
N_frame = 8;

S = N_D + N_A + N_C;

for idx = 1:N_frame
    frames(:,:,idx) = Data_2((idx-1)*S+2:idx*S+1,:);
end

D_data(:,:,:) = frames(1:N_D,:,:);
A_data(:,:,:) = frames(N_D+1:N_D+N_A,:,:);
C_data(:,:,:) = frames(N_D+N_A+1:S,:,:);

F_D = zeros(4,4,N_frame);
F_A = zeros(4,4,N_frame);
C_Expected = zeros(3,N_c,N_frame);

for idx = 1:N_frame
    F_D(:,:,idx) = Calculate_FD(Data_2,D_data(:,:,idx),N_D);

    A = A_data(:,:,idx);

    A_sum = zeros(1,3);
    for jdx = 1:N_A
        A_sum = A_sum + A(jdx,:);
    end
    A_bar = A_sum/N_A;
    
    A_mod = A;
    for jdx=1:N_A
        A_mod(jdx,:) = A(jdx,:) - A_bar;
    end
    
    M = zeros(4*N_A,4);
    for jdx = 1:length(a_mod(:,1))
        a = a_mod(jdx,:)';
        b = A_mod(jdx,:)';
        
        M((jdx-1)*4+1:jdx*4,:) = [0 (b-a)';...
                      (b-a) ssm(b+a)];
    end
    
    [U_A,S_A,V_A] = svd(M);
    
    Quat_A = quaternion(V_A(:,end)');
    R_A = quat2rotm(Quat_A);
    
    p = A_bar' - R_A*a_bar';
    
    F_A(:,:,idx) = [R_A p;...
                  0 0 0 1];

    F_DA = F_D(:,:,idx)\F_A(:,:,idx);
    for jdx =1:N_c
        C_Expected(:,jdx,idx) = F_DA(1:3,4) + F_DA(1:3,1:3)*c_data(jdx,:)';
    end
    C_Output(:,:,idx) = C_Expected(:,:,idx)';
end

function file_data = Import_data_from_file(file_name)
    % Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 3);
    
    % Specify range and delimiter
    opts.DataLines = [1, Inf];
    opts.Delimiter = ",";
    
    % Specify column names and types
    opts.VariableNames = ["VarName1", "VarName2", "VarName3"];
    opts.VariableTypes = ["double", "double", "double"];
    
    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";
    
    % Import the data
    file_table = readtable(file_name, opts);
    
    % Convert to output type
    file_data = table2array(file_table);
    
    % Clear temporary variables
    clear opts
end