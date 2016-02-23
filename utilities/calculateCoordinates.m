function [longitude, latitude] = calculateCoordinates()
    latitude = zeros(2400,1);
    longitude = zeros(4800,1);

    for i = 1:4800
        longitude(i) = -6 + (1/120) * (i-1);
    end
    longitude = repmat(longitude',2400,1);

    for j = 1:2400
        latitude(j) = 70 - (1/120) * (j-1);
    end
    latitude = repmat(latitude,1,4800);
end