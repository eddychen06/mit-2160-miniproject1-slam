function [value,location] = find_closest_element(test_value,vector)
%FIND_CLOSEST_ELEMENT Given a vector "vector" and a float value
%"test_value", this finds the element of "vector" closest to "test_value",
%and returns that value and its index in the vector

[~,location] = min(abs(test_value - vector));
value = vector(location);

end

