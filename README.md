Simulink-RTL-SDR
================

The Simulink-RTL-SDR project is an Open Source Software Package that enables owners of DVB-T dongles with an embedded Realtek RTL2832U chip-set to build models in Simulink that interface with the device in real-time. Simulink-RTL-SDR uses the *rtl-sdr* library and some code fragments from *OsmoSDR* to receive IQ samples from the DVB-T dongle. It is thus possible to build a wide variety of signal processing and wireless communication applications directly in Simulink while being able to test them on real hardware at the same time.

The Simulink-RTL-SDR project was initiated at the *Communication Engineering Lab (CEL)* at the *Karlsruhe Institute of Technology (KIT)*, Germany, <http://www.cel.kit.edu>.

Requirements
------------

- MATLAB/Simulink (R2009b or newer) and *MEX* compatible [compiler](http://www.mathworks.de/support/compilers)

- *rtl-sdr* library from the [OsmoSDR Wiki](http://sdr.osmocom.org/trac/wiki/rtl-sdr "rtl-sdr project page")

- Windows only: *POSIX Threads for Win32* from the [Project Homepage](http://sourceware.org/pthreads-win32/)

The Simulink-RTL-SDR software package doesn't come with precompiled binaries due to license issues, but the compilation is quite simple. 

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

   In case of something like `usb_claim_interface error -6`, you have to unload the dvb-t module driver which blocks the rtl-sdr dongle

   		$ sudo rmmod dvb_usb_rtl28xxu 

   Alternatively, add this module to your blacklist.

3. Get the Simulink-RTL-SDR source from the [GitHub](https://github.com/kit-cel/simulink-rtl-sdr) project page

		$ git clone git://github.com/kit-cel/simulink-rtl-sdr.git
        
	If you aren't using *git*, you can download a [compressed file](https://github.com/kit-cel/simulink-rtl-sdr) from the project page directly. Extract the archive to any folder you want.
		
4. Run MATLAB, switch to your Simulink-RTL-SDR directory and start the build process

		>> make.m

5. Add the *bin* and the *blockset* directory to the MATLAB path environment.

6. You will now find a new Toolbox named Simulink-RTL-SDR in the *Simulink Library Browser*. Additionally, a simple spectrum scope model is located in the directory *demo*.


Build/Install instructions for Microsoft Windows
------------------------------------------------

1. Get and build the *rtl-sdr* library. See the [OsmoSDR Wiki](http://sdr.osmocom.org/trac/wiki/rtl-sdr) for instructions or get [pre-build binaries](http://sdr.osmocom.org/trac/raw-attachment/wiki/rtl-sdr/RelWithDebInfo.zip). 

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

5. Get *pthreads for Windows* from their download page:

		ftp://sourceware.org/pub/pthreads-win32/ 


	Version 2.9.1 has been tested successfully 

		ftp://sourceware.org/pub/pthreads-win32/pthreads-w32-2-9-1-release.zip

6. Get the Simulink-RTL-SDR source from the [GitHub](https://github.com/kit-cel/simulink-rtl-sdr) project page

		$ git clone git://github.com/kit-cel/simulink-rtl-sdr.git

	If you aren't using *git*, you can download a [compressed file](https://github.com/kit-cel/simulink-rtl-sdr) from the project page directly. Extract the archive to any folder you want.

7. Run MATLAB and setup the *MEX* compiler
	
		>> mex -setup

8. Switch to your Simulink-RTL-SDR directory.

9. Modify make.m to point to the *rtl-sdr* include and lib directory. If you are using a 64-bit MATLAB, make sure to use a 64-bit build of the *rtl-sdr* library.

		RTL_SDR_INC_DIR = <path to rtl-sdr.h>
		RTL_SDR_LIB_DIR = <path to rtl-sdr.lib>

	Also adjust the include and lib paths for the *pthreads* library. As with the *rtl-sdr* library, make sure you use the x64 folder if your using the 64-bit version of MATLAB.

		PTHREAD_INC_DIR = <path to pthread.h>
		PTHREAD_LIB_DIR = <path to pthreadVC2.lib>

10. Start the build process (in MATLAB)

		>> make.m

11. Add the *bin* and the *blockset* directory to the MATLAB path environment.

12. Place the required DLLs in the *bin* directory of Simulink-RTL-SDR. If you downloaded the release versions of the *rtl-sdr* and *pthreads* libraries, simply copy all included dlls into the *bin* directory. Again, make sure you copy the x64 versions if your using 64-bit MATLAB.

13. You will now find a new Toolbox named Simulink-RTL-SDR in the *Simulink Library Browser*. Additionally, a simple spectrum scope model is located in the directory *demo*.

Weblinks
--------

The following table lists various weblinks which might be useful to you:

- [rtl-sdr](http://sdr.osmocom.org/trac/wiki/rtl-sdr) - Information about the DVB-T dongles with an Realtek RTL2832U chipset and software to interface them.
- [Communication Engineering Lab (CEL)](http://www.cel.kit.edu/english/index.php) - The institute at the Karlsruhe Institute of Technology, Germany, that initiated the Simulink-RTL-SDR project.
- [Karlsruhe Institute of Technology (KIT)](http://www.kit.edu/english/index.php) - Karlsruhe Institute of Technology (KIT), University of the State Baden-Württemberg and National Large-scale Research Center of the Helmholtz-Association.

Copyright
---------

The Simulink-RTL-SDR interface consists of software from several authors under different licenses. The following table lists all included software packages:

- *Simulink-RTL-SDR*  
  Authors: Communication Engineering Lab (CEL), Karlsruhe Institute of Technology (KIT), Michael Schwall, Sebastian Koslowski  
  License: GNU General Public License  
  Source:  simulink-rtlsdr/
- *rtl-sdr*  
  Authors: Steve Markgraf, Dimitri Stolnikov, Hoernchen  
  License: GNU General Public License  
  Source:  wherever you saved it

Contact and Support
-------------------

If you have further questions concerning the Simulink-RTL-SDR interface (e.g. the build-process) or you've found a bug, please feel free to contact us:

- Michael Schwall - [michael.schwall@kit.edu](mailto:michael.schwall@kit.edu)
- Sebastian Koslowski - [sebastian.koslowski@kit.edu](mailto:sebastian.koslowski@kit.edu)

Please provide the following information when requesting support:

- Operating System (Linux/Windows, 32-bit/64-bit, Linux-Distribution, ...)
- MATLAB version (R20***, 32-bit/64-bit, ...)
- Compiler and version (GCC x.x.x, MVSC, ...)
- Detailed error report (MATLAB console output or GUI screen-shot)
- ...and all other information you consider as meaningful ;)

Change-log
---------

- v1.2: Moved project information from cel.kit.edu to GitHub

- v1.1: Switched to async mode for sample reading

- v1.0: first release of the Simulink-RTL-SDR software package