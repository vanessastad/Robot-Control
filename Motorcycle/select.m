function selector = select(a)

if a == 1
    fprintf('Choose your reference: \n');
    fprintf('1 - Constant \n');
    fprintf('2 - Sinusoidal \n');
    fprintf('3 - Parabolic \n');
    
    while(1)
    selector = input('? ');
        if (selector >= 1 && selector <= 3)
             break;
        end
    end
    
elseif a == 2
    fprintf('\n Choose your system: \n');
    fprintf('1 - Computed torque \n');
    fprintf('2 - EKF \n');
    fprintf('3 - Li-Slotine \n');
    
    while(1)
    selector = input('? ');
        if (selector >= 1 && selector <= 3)
             break;
        end
    end
end