% ######################################################################################################################
%
% ===============
% The QSS toolbox
% ===============
%
%
% 1. Hardware requirements
% ------------------------
%   - No specific hardware component is requested.
%
%
% 2. Software requirements
% ------------------------
%   - Matlab 7 (R14) or later.
%   - Simulink 6 or later.
%
%
% 3. Structure of the QSS toolbox
% -------------------------------
%   QSS_TB................................................The main folder 
%       |--> Data.........................................All necessary data for the whole QSS TB are saved here
%              |--> DrivingCycles.........................Data of the different driving cycles
%                      |--> Europe
%                      |--> Japan
%                      |--> USA
%       |--> Documentation................................The QSS TB manual is here
%       |--> Examples.....................................Some examples to show how the QSS TB works
%              |--> Data..................................All necessary data to run the examples are saved here
%                      |--> EnergyConverter...............Data of the different energy converters
%                              |--> CombustionEngine
%                              |--> ElectricGenerator
%                              |--> ElectricMotor
%                              |--> FuelCell
%              |--> Intro.................................Please refer to the corresponding README file
%              |--> OptimalTransmissionDesign.............Please refer to the corresponding README file
%              |--> SeriesHybridVehicle...................Please refer to the corresponding README file
%       |--> Functions....................................All necessary functions for the whole QSS TB are saved here
%       |--> Initialize...................................Files to initialize the Simulink blocks of the QSS TB library
%       |--> Library......................................The Simulink blocks of the QSS TB library are here      
%
%
% 4. Getting started with the QSS toolbox
% ---------------------------------------
%	- Copy the folder "QSS_TB" on your computer.
%	- Add the folder "QSS_TB" and all its subfolders to the Matlab path.
%	- Type "qss_tb_library" to open the QSS TB library in the Matlab Command Window.
%   - Enjoy!
%
% ######################################################################################################################
