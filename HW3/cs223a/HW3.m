syms d2 s13 c13 s1 c1 
A = [1 0 1;...
    -d2*c1 - 2*s13 -s1 -2*s13;...
    -d2*s1 + 2*c13 c1 2*c13];
b = [1;0;0];
q = A\b

L1 = 2;
L2 = 1;
L3 = 1;
L(1) = Revolute('d', L1, 'a', 0, 'alpha', 0);
L(2) = Revolute('d', 0, 'a', 1, 'alpha', pi/2);
L(3) = Revolute('d', 0, 'a', L2, 'alpha', -pi/2);
L(4) = Revolute('d', 0, 'a', L3, 'alpha', 0);
% for i = 1:4
%     L(i).mdh = 1;
% end
rbt = SerialLink(L, 'name', 'A');

q = [0,pi/4*0,0,0];
rbt.plot(q)
