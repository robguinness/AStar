function [pC,sAP,tAP,pL] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath,pathLength, out_path)
%saveResults(pathCoordinates,speedAlongPath,timeAlongPath) saves the
%results obtained in the course of optimization process carried out by
%AStar algorithm

%   The function stores the obtained results in a folder called 'results'.
%   The following variables are stored:
%       - path coordinates (pC)
%       - speed along path (sAP)
%       - time along path (tAP)

%t=datestr(now,'mmddyyyy_HHMM');

pC=array2table([pathCoordinates,speedAlongPath'],'VariableNames',{'Long' 'Lat' 'Speed'}); 
writetable(pC,strcat(out_path,'/OUTpathCoordinates.txt'));

sAP=array2table(speedAlongPath','VariableNames',{'knots'}); 
writetable(sAP,strcat(out_path,'/OUTspeedAlongPath.txt'));

tAP=array2table(timeAlongPath','VariableNames',{'hours'}); 
writetable(tAP,strcat(out_path,'/OUTtimeAlongPath.txt'));

pL=array2table(pathLength,'VariableNames',{'Nautical_miles'}); 
writetable(pL,strcat(out_path,'/OUTpathLength.txt'));

end