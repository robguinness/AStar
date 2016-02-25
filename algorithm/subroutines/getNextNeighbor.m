function neighbor = getNextNeighbor(j, currentNode)
    switch j
        case 1
                neighbor.x = currentNode.x;
                neighbor.y = currentNode.y+1;
        case 2
                neighbor.x = currentNode.x;
                neighbor.y = currentNode.y-1;
        case 3
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y;
        case 4
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y;
        case 5
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y+1;
        case 6
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y-1;                            
        case 7
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y-1;   
        case 8
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y+1;
        case 9
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y+2;                            
        case 10
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y-2;   
        case 11
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y+2; 
        case 12
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y-2;
        case 13
                neighbor.x = currentNode.x+2;
                neighbor.y = currentNode.y+1;
        case 14
                neighbor.x = currentNode.x+2;
                neighbor.y = currentNode.y-1;
        case 15
                neighbor.x = currentNode.x-2;
                neighbor.y = currentNode.y+1;
        case 16
                neighbor.x = currentNode.x-2;
                neighbor.y = currentNode.y-1;                            
        case 17
                neighbor.x = currentNode.x+3;
                neighbor.y = currentNode.y+1;   
        case 18
                neighbor.x = currentNode.x+3;
                neighbor.y = currentNode.y-1;
        case 19
                neighbor.x = currentNode.x-3;
                neighbor.y = currentNode.y+1;     
        case 20
                neighbor.x = currentNode.x-3;
                neighbor.y = currentNode.y-1;
        case 21
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y+3;  
        case 22
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y-3;  
        case 23
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y+3;  
        case 24
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y-3;  
        case 25
                neighbor.x = currentNode.x+2;
                neighbor.y = currentNode.y+3;  
        case 26
                neighbor.x = currentNode.x+2;
                neighbor.y = currentNode.y-3;  
        case 27
                neighbor.x = currentNode.x-2;
                neighbor.y = currentNode.y+3;  
        case 28
                neighbor.x = currentNode.x-2;
                neighbor.y = currentNode.y-3;  
        case 29
                neighbor.x = currentNode.x+3;
                neighbor.y = currentNode.y+2;  
        case 30
                neighbor.x = currentNode.x+3;
                neighbor.y = currentNode.y-2;  
        case 31
                neighbor.x = currentNode.x-3;
                neighbor.y = currentNode.y+2;  
        case 32
                neighbor.x = currentNode.x-3;
                neighbor.y = currentNode.y-2;  
        case 33
                neighbor.x = currentNode.x+4;
                neighbor.y = currentNode.y+5;  
        case 34
                neighbor.x = currentNode.x+4;
                neighbor.y = currentNode.y-5;  
        case 35
                neighbor.x = currentNode.x-4;
                neighbor.y = currentNode.y+5;  
        case 36
                neighbor.x = currentNode.x-4;
                neighbor.y = currentNode.y-5;
        case 37
                neighbor.x = currentNode.x+5;
                neighbor.y = currentNode.y+4;  
        case 38
                neighbor.x = currentNode.x+5;
                neighbor.y = currentNode.y-4;  
        case 39
                neighbor.x = currentNode.x-5;
                neighbor.y = currentNode.y+4;  
        case 40
                neighbor.x = currentNode.x-5;
                neighbor.y = currentNode.y-4;
        case 41
                neighbor.x = currentNode.x+5;
                neighbor.y = currentNode.y+1;  
        case 42
                neighbor.x = currentNode.x+5;
                neighbor.y = currentNode.y-1;  
        case 43
                neighbor.x = currentNode.x-5;
                neighbor.y = currentNode.y+1;  
        case 44
                neighbor.x = currentNode.x-5;
                neighbor.y = currentNode.y-1;
        case 45
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y+5;  
        case 46
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y-5;  
        case 47
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y+5;  
        case 48
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y-5;
        case 49
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y+10;  
        case 50
                neighbor.x = currentNode.x+1;
                neighbor.y = currentNode.y-10;  
        case 51
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y+10;  
        case 52
                neighbor.x = currentNode.x-1;
                neighbor.y = currentNode.y-10;           
        case 53
                neighbor.x = currentNode.x+10;
                neighbor.y = currentNode.y+1;  
        case 54
                neighbor.x = currentNode.x+10;
                neighbor.y = currentNode.y-1;  
        case 55
                neighbor.x = currentNode.x-10;
                neighbor.y = currentNode.y+1;  
        case 56
                neighbor.x = currentNode.x-10;
                neighbor.y = currentNode.y-1;                     
        otherwise
            disp('Error!')
    end
end