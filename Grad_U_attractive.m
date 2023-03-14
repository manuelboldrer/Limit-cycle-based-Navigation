function Grad_U = Grad_U_attractive(e_q_x,e_q_y,k_a)
%Grad_U_attractive
%    Grad_U = Grad_U_attractive(E_Q_X,E_Q_Y,K_A)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    14-Mar-2023 14:56:33

t2 = abs(e_q_x);
t3 = abs(e_q_y);
t4 = t2.^2;
t5 = t3.^2;
t6 = t4+t5;
t7 = 1.0./sqrt(t6);
Grad_U = [k_a.*t2.*t7.*sign(e_q_x);k_a.*t3.*t7.*sign(e_q_y)];
