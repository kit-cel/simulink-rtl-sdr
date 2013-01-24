/*
* Copyright 2012 Communications Engineering Lab, KIT
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING. If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

/*
    Find rtl-sdr devices attached to the host.
*/

/* Simulink includes */
#include <simstruc.h>
#include <mex.h>

/* rtl-sdr includes */
#include <rtl-sdr.h>

/* Entry point to C/C++ */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    int_T i, j, ret;
    char vendor[256], product[256], serial[256];
    int gains[100];

    /* allocate memory for rtl-sdr object */
    rtlsdr_dev_t *_device = (rtlsdr_dev_t*)malloc(8);    

    /* ================================================================= */
    /* check input and ouput
    /* ================================================================= */
            
    if (nrhs != 0) {
        mexErrMsgTxt("No arguments required.");
    }            
    
    /* ================================================================= */
    /* find devices
    /* ================================================================= */
    
    mexPrintf("\n");

    /* get number of connected devices */
    int_T device_count = rtlsdr_get_device_count();
    if (!device_count) {
        mexPrintf("No supported devices found.\n");
        return;
    }
    mexPrintf("Found %d device(s):\n", device_count);

    /* for each detected device */
    for (i = 0; i < device_count; i++) {
        ret = rtlsdr_get_device_usb_strings(i, vendor, product, serial);
        if (ret < 0) {
            mexErrMsgTxt("Failed to get rtl-sdr device strings.");
        }

        mexPrintf("\nDevice Index %d\n",i);        
        mexPrintf("  Vendor: \t%s\n", vendor);
        mexPrintf("  Product: \t%s\n", product);
        mexPrintf("  Serial: \t%s\n", serial);

        /* open rtl-sdr device */
        ret = rtlsdr_open(&_device, i);
        if (ret < 0) {
            mexErrMsgTxt("Failed to open rtl-sdr device.");
        }

        /* get number of gains and values */
        ret = rtlsdr_get_tuner_gains(_device, gains);
        if (ret <= 0) {
            rtlsdr_close(_device);
            mexErrMsgTxt("Failed to get rtl-sdr Tuner gains.");
        }
        mexPrintf("  Gains [dB]:\t", ret);

        for (j = 0; j < ret; j++)
            mexPrintf("%.1f ", gains[j] / 10.0);
        mexPrintf("\n");

        /* get Tuner type */
        switch(rtlsdr_get_tuner_type(_device))
        {
            case RTLSDR_TUNER_E4000:
                mexPrintf("  Tuner: \tE4000\n");
                break;
            case RTLSDR_TUNER_FC0012:
                mexPrintf("  Tuner: \tFC0012\n");
                break;
            case RTLSDR_TUNER_FC0013:
                mexPrintf("  Tuner: \tFC0013\n");
                break;
            case RTLSDR_TUNER_FC2580:
                mexPrintf("  Tuner: \tFC2580\n");
                break;
            default: 
                mexPrintf("  Tuner: \tunknown\n");
        }
        mexPrintf("\n");

        rtlsdr_close(_device);
    }
}
