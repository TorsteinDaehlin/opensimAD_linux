function [] = buildExternalFunction(fooPath, outputFilename, outputDir, compiler, verbose_mode)
% --------------------------------------------------------------------------
% buildExternalFunction
%   Compiles the expression graph (foo_jac.c) into an external function
%
%
% INPUT:
%   - fooPath -
%   * path to foo.py and foo_jac.c [char]
%
%   - outputFilename -
%   * name of the generated file [char]
%
%   - outputDir -
%   * full path to directory where the generated file should be saved [char]
%
%   - compiler -
%   * command prompt argument for the compiler. [char]
%   Example inputs:
%       Visual studio 2015: 'Visual Studio 14 2015 Win64'
%       Visual studio 2017: 'Visual Studio 15 2017 Win64'
%       Visual studio 2017: 'Visual Studio 16 2019'
%       Visual studio 2017: 'Visual Studio 17 2022'
%
%   - verbose_mode -
%   * outputs from windows command prompt are printed to matlab command 
%   window if true. [bool]
%
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

%% set paths
[pathUtilities, ~, ~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
if ispc
    path_expression_graph_filename_build = fullfile(pathMain, 'buildExpressionGraph', 'windows', outputFilename);
elseif isunix
    path_expression_graph_filename_build = fullfile(pathMain, 'buildExpressionGraph', 'linux', outputFilename);
elseif ismac
    path_expression_graph_filename_build = fullfile(pathMain, 'buildExpressionGraph', 'macOS', outputFilename);
end
path_external_functions_filename_install = fullfile(pathMain, 'installExternalFunction', outputFilename);
mkdir(path_external_functions_filename_install);
path_external_functions_build = fullfile(pathMain, 'buildExternalFunction');
path_external_functions_filename_build = fullfile(pathMain, 'buildExternalFunction', outputFilename);
mkdir(path_external_functions_filename_build);

%% use cmake to compile foo_jac.c to .dll
copyfile(fullfile(path_external_functions_build,'CMakeLists.txt'), path_external_functions_filename_build)
copyfile(fullfile(fooPath,'foo_jac.c'), path_external_functions_filename_build)
cd(path_external_functions_filename_build);

if ispc
    cmd1 = ['cmake "', path_external_functions_filename_build, '" -G "', compiler, '" -DTARGET_NAME:STRING="',...
        outputFilename, '" -DINSTALL_DIR:PATH="', path_external_functions_filename_install, '"'];
    cmd2 = 'cmake --build . --config RelWithDebInfo --target install';
elseif isunix
    cmd1 = ['cmake "' path_external_functions_filename_build '" -DTARGET_NAME:STRING="' outputFilename ...
        '" -DINSTALL_DIR:PATH="' path_external_functions_filename_install '"'];
    cmd2 = "make install";
end

if verbose_mode
    system(cmd1);
else
    [~,~] = system(cmd1);
end

if verbose_mode
    system(cmd2);
else
    [~,~] = system(cmd2);
end
cd(pathMain);

%% copy generated files to output directory
if ispc
    copyfile(fullfile(path_external_functions_filename_install, 'bin', [outputFilename '.dll']), outputDir);
    copyfile(fullfile(path_external_functions_filename_install, 'lib', [outputFilename '.lib']), outputDir);
elseif isunix
    movefile(fullfile(path_external_functions_filename_install, 'lib', ['lib' outputFilename '.so']), ...
        fullfile(outputDir, [outputFilename '.so']));
end

%% remove directories with temporary files
rmdir(path_expression_graph_filename_build, 's');
rmdir(path_external_functions_filename_install, 's');
rmdir(path_external_functions_filename_build, 's');

end