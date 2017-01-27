function [pC,sAP,tAP,pL] = saveResults(pathCoordinates,speedAlongPath,timeAlongPath,pathLength)
%saveResults(pathCoordinates,speedAlongPath,timeAlongPath) saves the
%results obtained in the course of optimization process carried out by
%AStar algorithm

%   The function stores the obtained results in a folder called 'results'.
%   The following variables are stored:
%       - path coordinates (pC)
%       - speed along path (sAP)
%       - time along path (tAP)

t=datestr(now,'mmddyyyy_HHMM');

pC=array2table([pathCoordinates,speedAlongPath'],'VariableNames',{'Long' 'Lat' 'Speed'}); 
writetable(pC,['results/' t '_OUTpathCoordinates.txt']);

sAP=array2table(speedAlongPath','VariableNames',{'knots'}); 
writetable(sAP,['results/' t '_OUTspeedAlongPath.txt']);

tAP=array2table(timeAlongPath','VariableNames',{'hours'}); 
writetable(tAP,['results/' t '_OUTtimeAlongPath.txt']);

pL=array2table(pathLength,'VariableNames',{'Nautical_miles'}); 
writetable(pL,['results/' t '_OUTpathLength.txt']);

end