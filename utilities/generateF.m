    function [] = generateF(nInputs, fooPath)
% --------------------------------------------------------------------------
% generateF
%   Generates an expression graph of the function and its derivative
%   (foo_jac.c) based on the expression graph (foo.py).
%
%
% INPUT:
%   - nInputs -
%   * number of input arguments for the external function [double]
%
%   - fooPath -
%   * path to foo.py [char]
%
%   - secondOrderDerivatives -
%   * do you want to calculate 2nd derivatives of external function outputs 
%   w.r.t. inputs? [bool]
%
% OUTPUT:
%   - (This function does not return output arguments) -
%
% Reference: 
%   Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation 
%   improves the computational efficiency of OpenSim-based trajectory 
%   optimization of human movement. PLoS ONE 14(10): e0217730. 
%   https://doi.org/10.1371/journal.pone.0217730
%
% Original author: Lars D'Hondt (based on code by Antoine Falisse)
% Original date: 8/May/2023
%
% Last edit by: Torstein E Daehlin
% Last edit date: 10/April/2025
% --------------------------------------------------------------------------


% We use MATLABs built-in functionality for calling python code to generate
% the external funcition from foo.py. 


[pathUtilities, ~, ~] = fileparts(mfilename('fullpath'));

nInputs = int16(nInputs); % Ensures nInputs is the correct type as passing it as a double causes an error in python

cd(fooPath);
cmd = sprintf(['python ' fullfile(pathUtilities, 'genF.py') ' %i %s'], ...
    nInputs, fooPath);
system(cmd);

cd(pathUtilities);

end