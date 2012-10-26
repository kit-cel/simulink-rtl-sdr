%
% Copyright 2010 Communications Engineering Lab, KIT
%
% This is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3, or (at your option)
% any later version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software; see the file COPYING. If not, write to
% the Free Software Foundation, Inc., 51 Franklin Street,
% Boston, MA 02110-1301, USA.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make script for Simulink-RTL-SDR
% use "make -v" to get a verbose output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function make(varargin)

RTL_SDR_BIN_DIR = fullfile(pwd,'bin');

if ispc
    % this should point to the directory of rtl-sdr.h
    RTL_SDR_INC_DIR = fullfile(pwd,'deps','rtl-sdr-release');
    % this should point to the directory of rtlsdr.lib
    RTL_SDR_LIB_DIR = fullfile(pwd,'deps','rtl-sdr-release','x64');
    
    % this should point to the directory of pthread.h
    PTHREAD_INC_DIR = fullfile(pwd,'deps','pthread','include');
    % this should point to the directory of pthreadVC.lib
    PTHREAD_LIB_DIR = fullfile(pwd,'deps','pthread','lib','x64');
    
    % make sure the other required DLLS are in your PATH 
    % (e.g. place them in the bin directory)
    
    options = { ...
        ['-I' pwd]; ...
        ['-I' RTL_SDR_INC_DIR]; ...
        ['-L' RTL_SDR_LIB_DIR]; ...
        ['-l' 'rtlsdr']; ...
    };
    options_pthread = { ...
        ['-I' PTHREAD_INC_DIR]; ...
        ['-L' PTHREAD_LIB_DIR]; ...
        ['-l' 'pthreadVC2'] ...
    };
elseif isunix
    options = { ...
        ['-l' 'rtlsdr']
    };
    options_pthread = { ...
        ['-l' 'pthread'] ...
    };
else
    error('Platform not supported')
end

% create bin order if not exist
if (~exist(RTL_SDR_BIN_DIR,'dir'))
    mkdir(RTL_SDR_BIN_DIR);
end

% add command line args if exist
if ~isempty(varargin)
    options = [options; char(varargin)];
end

% Set path hint

% compile source and find_devices
fprintf('\nCompiling rtlsdr_source.cpp ... ');
mex(options{:},options_pthread{:},'-outdir',RTL_SDR_BIN_DIR,'src/rtlsdr_source.cpp')
fprintf('Done.\n');

fprintf('\nCompiling rtlsdr_find_devices.cpp ... ');
mex(options{:},'-outdir',RTL_SDR_BIN_DIR,'src/rtlsdr_find_devices.cpp')
fprintf('Done.\n');

fprintf('\nCompiling rtlsdr_dev.cpp ... ');
mex(options{:},'-outdir',RTL_SDR_BIN_DIR,'src/rtlsdr_dev.cpp')
fprintf('Done.\n');

% copy help file
copyfile(fullfile(pwd,'src','rtlsdr_dev.m'),fullfile(RTL_SDR_BIN_DIR,'rtlsdr_dev.m'));
    

% Set path hint
fprintf('\nBuild successful.\n\nSet path to:\n -> %s\n -> %s\n',RTL_SDR_BIN_DIR,fullfile(pwd,'blockset'));