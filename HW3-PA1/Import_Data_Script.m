clear all; clc;
File_ID = fopen("HW3-PA1\pa1-debug-a-calbody.txt");
format_spec = '%d %f';
sizeA = [];
A = fscanf(File_ID,format_spec,sizeA);
fclose(File_ID);