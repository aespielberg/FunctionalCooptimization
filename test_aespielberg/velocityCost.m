function [g,dg] = velocityCost(T, x, params)
                %TODO: make this not have a hardcoded 23 somehow
                g = (x(1)) / T;
                dg = [-1/T^2, 1/T, zeros(1, length(x) - 1), zeros(1, length(params))];
                
                return;
                
            end