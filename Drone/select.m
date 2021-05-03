function selector = select(a)

if a == 1
    fprintf('Choose your reference: \n');
    fprintf('1 - Circle \n');
    fprintf('2 - Eight curve \n');
    fprintf('3 - Hypocycloid \n');
    
    while(1)
    selector = input('? ');
        if (selector >= 1 && selector <= 3)
             break;
        end
    end
end