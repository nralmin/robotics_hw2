% Narmin Aliyeva 2177269
function [robotVel] = hw3_script3(wheelSp)
   % function taking three wheel rotational speeds as a column vector,
   % returning the associated robot velocities as a column vector
   
   syms x(t) y(t) theta(t)
   OTF = transl2(x,y) * rotz(theta);
   FRO = rotz(-theta);
   
   syms D
   FpA = [D/2; D*sqrt(3)/2];
   FpB = [-D; 0];
   FpC = [D/2; -D*sqrt(3)/2];
   
   pA = OTF * [FpA; 1];
   pB = OTF * [FpB; 1];
   pC = OTF * [FpC; 1];
   
   pA_dot = diff(pA);
   pB_dot = diff(pB);
   pC_dot = diff(pC);
   
   APF = [-cos(30*pi/180), cos(60*pi/180), D];
   BPF = [0, -1, D];
   CPF = [cos(30*pi/180), cos(60*pi/180), D];
   
   syms R
   robot2wheel = 1/R * [APF; BPF; CPF] * FRO;
   wheel2robot = robot2wheel^-1;
   
   robotVel = wheel2robot * wheelSp;
   robotVel = simplify(robotVel);
end
   