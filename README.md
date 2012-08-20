Simulink-RTL-SDR
================

The Simulink-RTL-SDR project is an Open Source Software Package that enables owners of DVB-T dongles with an embedded Realtek RTL2832U chipset to build models in Simulink that interface with the device in realtime. Simulink-RTL-SDR uses the *rtl-sdr* library from *OsmoSDR* to receive IQ samples from the DVB-T dongle. It is thus possible to build a wide variety of signal processing and wireless communication applications directly in Simulink while being able to test them on real hardware at the same time.

The Simulink-RTL-SDR project was initiated at the *Communication Engineering Lab (CEL)* at the *Karlsruhe Institute of Technology (KIT)*, Germany, <http://www.cel.kit.edu>.

For more information, please visit the Simulink-RTL-SDR [project page](http://www.cel.kit.edu/simulink_rtl_sdr.php).

Requirements
------------

- MATLAB/Simulink (R2009b or newer) and *MEX* compatible compiler

- *rtl-sdr* library from the [OsmoSDR Wiki](http://sdr.osmocom.org/trac/wiki/rtl-sdr "rtl-sdr project page")

The Simulink-rtl-sdr software package doesn't come with precompiled binaries due to license issues, but the compilation is quite simple. 

Build/Install instructions for Linux
------------------------------------

1. Get, build and install the *rtl-sdr* library. See the [OsmoSDR Wiki](http://sdr.osmocom.org/trac/wiki/rtl-sdr) for instructions.

2. Plug in your device and run the included test application

		$ rtl_test
		Found 1 device(s):
          0:  Generic RTL2832U (e.g. hama nano)
        Using device 0: Generic RTL2832U (e.g. hama nano)
        Found Elonics E4000 tuner
        Supported gain values (18): -1.0 1.5 4.0 6.5 9.0 11.5 14.0 16.5 19.0 21.5 24.0 29.0 34.0 42.0 43.0 45.0 47.0 49.0 
        Reading samples in async mode...

3. Get the Simulink-RTL-SDR source from the [GitHub](https://github.com/kit-cel/simulink-rtl-sdr) project page

		$ git clone git://github.com/kit-cel/simulink-rtl-sdr.git
If you aren't using *git*, you can download a [compressed file](https://github.com/kit-cel/simulink-rtl-sdr) from the project page directly. Extract the archive to any folder you want.
		
4. Run MATLAB, switch to your Simulink-RTL-SDR directory and start the build process

		>> make.m

5. Add the *bin* and the *blockset* directory to the MATLAB path environment.

6. You will now find a new Toolbox named Simulink-RTL-SDR in the *Simulink Library Browser*. Additionally, a simple spectrum scope model is located in the directory *demo*.


Build/Install instructions for Microsoft Windows
------------------------------------------------

1. Get and build the *rtl-sdr* library. See the [OsmoSDR Wiki](http://sdr.osmocom.org/trac/wiki/rtl-sdr) for instructions or get [pre-build binaries](http://sdr.osmocom.org/trac/attachment/wiki/rtl-sdr/RelWithDebInfo.zip). 

2. Plug in your device, abort the automated search for drivers.

3. Get [Zadig](http://sourceforge.net/projects/libwdi/files/zadig/) and install the *WinUSB* driver for your device as shown on the [SDR# Homepage](http://rtlsdr.org/softwarewindows). If *Zadig* fails, you can download the [WinUSB drivers](http://libusb-winusb-wip.googlecode.com/files/winusb%20driver.zip) directly. Change the VID/PID in the inf-file to match your device before installing the driver through the *Device Manager*.

4. Test the *rtl-sdr* library with the included test application

		> rtl_test.exe
		Found 1 device(s):
          0:  Generic RTL2832U (e.g. hama nano)
        Using device 0: Generic RTL2832U (e.g. hama nano)
        Found Elonics E4000 tuner
        Supported gain values (18): -1.0 1.5 4.0 6.5 9.0 11.5 14.0 16.5 19.0 21.5 24.0 29.0 34.0 42.0 43.0 45.0 47.0 49.0 
        Reading samples in async mode...

5. Get the Simulink-RTL-SDR source from the [GitHub](https://github.com/kit-cel/simulink-rtl-sdr) project page

		$ git clone git://github.com/kit-cel/simulink-rtl-sdr.git
If you aren't using *git*, you can download a [compressed file](https://github.com/kit-cel/simulink-rtl-sdr) from the project page directly. Extract the archive to any folder you want.

6. Run MATLAB and setup the *MEX* compiler
	
		>> mex -setup

7. Switch to your Simulink-RTL-SDR directory.

8. Modify make.m to point to the *rtl-sdr* include and lib directory. If you are using a 64-bit MATLAB, make sure to use a 64-bit build of the *rtl-sdr* library.

		RTL_SDR_INC_DIR = <path to rtl-sdr.h>
		RTL_SDR_LIB_DIR = <path to rtl-sdr.lib>

9. Start the build process 

		>> make.m

10. Add the *bin* and the *blockset* directory to the MATLAB path environment.

11. Place the required DLLs (e.g. libusb-1.0.dll, msvcr100.dll and rtlsdr.dll) in the *bin* directory of Simulink-RTL-SDR.

12. You will now find a new Toolbox named Simulink-RTL-SDR in the *Simulink Library Browser*. Additionally, a simple spectrum scope model is located in the directory *demo*.

Changelog
---------

Version 1.0

- first release of the Simulink-RTL-SDR software package