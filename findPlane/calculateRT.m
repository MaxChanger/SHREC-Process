outfile = fopen('Initialize_RT.txt','w');

M=load('./pcd/B_normals.txt');
A=M(1:3,1:3);
D_A=-M(1:3,4);

% A(1,:)*A(2,:)'
% A(1,:)*A(3,:)'
% A(2,:)*A(3,:)'

N=load('./pcd/A_normals.txt');
B=N(1:3,1:3);
D_B=-N(1:3,4);

% B(1,:)*B(2,:)'
% B(1,:)*B(3,:)'
% B(2,:)*B(3,:)'

P_1=A^(-1)*D_A;
P_2=B^(-1)*D_B;
Po1=[P_1';P_1';P_1'];
Po2=[P_2';P_2';P_2'];

A=[A(1,:)/sqrt(A(1,1)^2+A(1,2)^2+A(1,3)^2);A(2,:)/sqrt(A(2,1)^2+A(2,2)^2+A(2,3)^2);A(3,:)/sqrt(A(3,1)^2+A(3,2)^2+A(3,3)^2)];
B=[B(1,:)/sqrt(B(1,1)^2+B(1,2)^2+B(1,3)^2);B(2,:)/sqrt(B(2,1)^2+B(2,2)^2+B(2,3)^2);B(3,:)/sqrt(B(3,1)^2+B(3,2)^2+B(3,3)^2)];

C=A+Po1;
D=B+Po2;

o1=[sum(C(:,1))/3 sum(C(:,2))/3 sum(C(:,3))/3];
o2=[sum(D(:,1))/3 sum(D(:,2))/3 sum(D(:,3))/3];

% stC1=std(C(:,1));
% stC2=std(C(:,2));
% stC3=std(C(:,3));
% 
% stD1=std(D(:,1));
% stD2=std(D(:,2));
% stD3=std(D(:,3));

C=C-[o1;o1;o1];
D=D-[o2;o2;o2];

%C=[C(:,1)/stC1 C(:,2)/stC2 C(:,3)/stC3];
%D=[D(:,1)/stD1 D(:,2)/stD2 D(:,3)/stD3];

C=C';
D=D';
H=C(:,1)*D(:,1)'+C(:,2)*D(:,2)'+C(:,3)*D(:,3)';
%H=C*D';

[U,S,V]=svd(H);
R=V*[1 0 0;0 1 0;0 0 det(V*U')]*U';
% T=P_2-R'*P_1;
T2=P_2-R*P_1;
T = T2';
%T2=P_2-P_1;


%% _the normal vector translated
%A_=[R*A(1,:)'+T2 R*A(2,:)'+T2 R*A(3,:)'+T2];
A_=[R*A(1,:)' R*A(2,:)' R*A(3,:)'];
index=[B(1,:)*A_(:,1) B(2,:)*A_(:,2) B(3,:)*A_(:,3)];

fprintf(outfile,'%f %f %f\n',R(1,1),R(1,2),R(1,3));
fprintf(outfile,'%f %f %f\n',R(2,1),R(2,2),R(2,3));
fprintf(outfile,'%f %f %f\n',R(3,1),R(3,2),R(3,3));
fprintf(outfile,'%f %f %f\n',T(1,1),T(1,2),T(1,3));
fclose(outfile);

