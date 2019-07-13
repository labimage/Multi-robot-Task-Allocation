clc;
clear;

%% test sequence2rank function
A=cell(4,3);
A{1,1}=[1,3,4]';
A{1,2}=5;
A{1,3}=[2,6,7]';
A{2,1}=1;
A{2,2}=[2,4,5]';
A{2,3}=3;
A{3,1}=6;
A{3,2}=1;
A{3,3}=[2,3,4,5]';
A{4,1}=[2,4]';
A{4,2}=3;
A{4,3}=1;

B=cell(4,3);
B{1,1}=[3,5,6]';
B{1,2}=6;
B{1,3}=[4,8,9]';
B{2,1}=1;
B{2,2}=[2,4,5]';
B{2,3}=3;
B{3,1}=7;
B{3,2}=1;
B{3,3}=[2,5,6,7]';
B{4,1}=[2,4]';
B{4,2}=3;
B{4,3}=1;

%% test sequence2rank function
C=sequence2rank(A,B);

%% test rank2sequence function
[D,E]=rank2sequence(C);

%% test objective function
P=cell(4,3);
P{1,1}=[10,1,1]';
P{1,2}=9;
P{1,3}=[1,3,4]';
P{2,1}=5;
P{2,2}=[5,1,1]';
P{2,3}=10;
P{3,1}=3;
P{3,2}=5;
P{3,3}=[5,1,1,9]';
P{4,1}=[5,1]';
P{4,2}=10;
P{4,3}=5;

global robotNum
global agentNum
global TaskNumMat
robotNum=4;
agentNum=3;
TaskNumMat=[3 1 3;1 3 1;1 1 4;2 1 1];

CVec=[C{1,1};C{1,2};C{1,3};C{2,1};C{2,2};C{2,3};C{3,1};C{3,2};C{3,3};C{4,1};C{4,2};C{4,3}];
Cmax=H2TS_fitness2(CVec,P)

