function [a, b] = getNextNeighbor(j, parentXval, parentYval)
    switch j
        case 1
                a = parentXval;
                b = parentYval+1;
        case 2
                a = parentXval;
                b = parentYval-1;
        case 3
                a = parentXval+1;
                b = parentYval;
        case 4
                a = parentXval-1;
                b = parentYval;
        case 5
                a = parentXval+1;
                b = parentYval+1;
        case 6
                a = parentXval+1;
                b = parentYval-1;                            
        case 7
                a = parentXval-1;
                b = parentYval-1;   
        case 8
                a = parentXval-1;
                b = parentYval+1;
        case 9
                a = parentXval+1;
                b = parentYval+2;                            
        case 10
                a = parentXval+1;
                b = parentYval-2;   
        case 11
                a = parentXval-1;
                b = parentYval+2; 
        case 12
                a = parentXval-1;
                b = parentYval-2;
        case 13
                a = parentXval+2;
                b = parentYval+1;
        case 14
                a = parentXval+2;
                b = parentYval-1;
        case 15
                a = parentXval-2;
                b = parentYval+1;
        case 16
                a = parentXval-2;
                b = parentYval-1;                            
        case 17
                a = parentXval+3;
                b = parentYval+1;   
        case 18
                a = parentXval+3;
                b = parentYval-1;
        case 19
                a = parentXval-3;
                b = parentYval+1;     
        case 20
                a = parentXval-3;
                b = parentYval-1;
        case 21
                a = parentXval+1;
                b = parentYval+3;  
        case 22
                a = parentXval+1;
                b = parentYval-3;  
        case 23
                a = parentXval-1;
                b = parentYval+3;  
        case 24
                a = parentXval-1;
                b = parentYval-3;  
        case 25
                a = parentXval+2;
                b = parentYval+3;  
        case 26
                a = parentXval+2;
                b = parentYval-3;  
        case 27
                a = parentXval-2;
                b = parentYval+3;  
        case 28
                a = parentXval-2;
                b = parentYval-3;  
        case 29
                a = parentXval+3;
                b = parentYval+2;  
        case 30
                a = parentXval+3;
                b = parentYval-2;  
        case 31
                a = parentXval-3;
                b = parentYval+2;  
        case 32
                a = parentXval-3;
                b = parentYval-2;  
        case 33
                a = parentXval+4;
                b = parentYval+5;  
        case 34
                a = parentXval+4;
                b = parentYval-5;  
        case 35
                a = parentXval-4;
                b = parentYval+5;  
        case 36
                a = parentXval-4;
                b = parentYval-5;
        case 37
                a = parentXval+5;
                b = parentYval+4;  
        case 38
                a = parentXval+5;
                b = parentYval-4;  
        case 39
                a = parentXval-5;
                b = parentYval+4;  
        case 40
                a = parentXval-5;
                b = parentYval-4;
        case 41
                a = parentXval+5;
                b = parentYval+1;  
        case 42
                a = parentXval+5;
                b = parentYval-1;  
        case 43
                a = parentXval-5;
                b = parentYval+1;  
        case 44
                a = parentXval-5;
                b = parentYval-1;
        case 45
                a = parentXval+1;
                b = parentYval+5;  
        case 46
                a = parentXval+1;
                b = parentYval-5;  
        case 47
                a = parentXval-1;
                b = parentYval+5;  
        case 48
                a = parentXval-1;
                b = parentYval-5;
        case 49
                a = parentXval+1;
                b = parentYval+10;  
        case 50
                a = parentXval+1;
                b = parentYval-10;  
        case 51
                a = parentXval-1;
                b = parentYval+10;  
        case 52
                a = parentXval-1;
                b = parentYval-10;           
        case 53
                a = parentXval+10;
                b = parentYval+1;  
        case 54
                a = parentXval+10;
                b = parentYval-1;  
        case 55
                a = parentXval-10;
                b = parentYval+1;  
        case 56
                a = parentXval-10;
                b = parentYval-1;                     
        otherwise
            disp('Error!')
    end
end