% function q_des = redundancy_resolution(M,B,q,Tsd, tol_w, tol_v) 

J = J_body(B,q);
w = sqrt(det(J*J'));

dq0 = k0 * (dw/dq)'; 