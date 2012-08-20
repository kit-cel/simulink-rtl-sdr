function blkStruct = slblocks
% Returns information about the rtl-sdr device
% to the Simulink library browser.
%
% @version:		1.0
% @author:		Michael Schwall <michael.schwall@kit.edu>
% @copyright:	Communications Engineering Lab, http://www.cel.kit.edu
% @license:		GNU General Public License, Version 3

% Information for the "Blocksets and Toolboxes" subsystem (findblib)
blkStruct.Name = sprintf('Simulink-RTL-SDR');
blkStruct.OpenFcn = 'rtlsdr';
blkStruct.MaskDisplay = 'disp(''Simulink-RTL-SDR'')';

% Information for the "Simulink Library Browser"
Browser(1).Library = 'rtlsdr';
Browser(1).Name    = 'Simulink-RTL-SDR';
Browser(1).IsFlat  = 0;

blkStruct.Browser = Browser;
clear Browser;

blkStruct.ModelUpdaterMethods.fhUpdateModel = @UpdateSimulinkBlocksHelper;