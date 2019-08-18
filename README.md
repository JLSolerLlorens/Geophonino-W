# Geophonino-W
Arduino source code of ACN and MCN.

Source code for GUI Geophonino-W application for Procesing.

Configuration files for Xbee modules.

PCB design in Gerber file format.

Enclosure designs in stl format.

Recoded data for system characterization and tests.

Geophonino-W data acquisition example video.


**Description of files content**

Geophonino_W_ACN.ino: Source code developed for Geophonino-W Acquisition Control Node (ACN). Arduino 1.8.1 IDE must be used to record this file in ACN modules.

Geophonino_W_MCN.ino: Source code developed for Geophonino-W Management Control Node (MCN). Arduino 1.8.1 IDE must be used to record this file in MCN modules.

Geophonino_W_MCN.pde: Source code developed for GUI Geophonino-W application for Procesing. Processing 2.1.2 sofware must be used to run this file. 

Video_GeophoninoW_DataAcquisitionExample: Example of how data acquisition measurement was made with Geophonino-W user interfaze.

XbeeConfigurationFiles folder: It contains XCTU configuration files for each Xbee module. XCTU 6.3.1.3 sofware designed by Digi inc. must be used to write Xbee modules configuration.

DataFiles_LaboratoryTests folder: This folder contains data recorded in each laboratory test. Geophonino-W data are stored in comma separated values files. First column corresponds to time in miliseconds relative to 0 and the following columns corresponds to data recorded in counts for each channel. Reftek data are stored in SESAME ASCII data format (saf).

Enclosure design for 3D Printers folder: It contains stl files that can be printed in any 3D printer.
