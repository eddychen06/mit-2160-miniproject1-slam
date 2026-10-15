function [value,location] = find_previous_element(test_value,vector)
%FIND_PREVIOUS_ELEMENT Given a vector "vector" and a float value
%"test_value", this finds the element of "vector" less than or equal to 
% "test_value" that is closest to "test_value",
%and returns that value and its index in the vector

compare_vector = abs(test_value - vector) + (test_value < vector)*1e10;
[~,location] = min(compare_vector);
value = vector(location);

end

