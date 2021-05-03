function [n, k, Omega_end] = checkdim( Omega0, O, dim)


k = 0;
n = 0;
c = 0;
Omega_end = Omega0; %span(dh)
while n ~= dim 
for i = 1+c : size(Omega_end,2)+c
    c = c + 1;
    Omega_new = [Omega_end O(i+1,:)'];
    if rank(Omega_end) == rank(Omega_new)
        Omega_end = Omega_new;
        if rank(Omega_new) == dim
            n = rank(Omega_new);
            k = i;
            break
        end
    else
        Omega_end = Omega_new;
    end
end
end







% k = 0;
% n = 0;
% Omega_end = Omega0; %span(dh)
% while n ~= 3 
% for i = 1 : size(Omega_end,2)
%     Omega_new = [Omega_end O(i+1,:)'];   
%     if rank(Omega_end) == rank(Omega_new) && rank(Omega_new) == dim 
%         n = rank(Omega_end);
%         k = i;
%         break
%     else
%         Omega_end = Omega_new;
%     end
% end
% end