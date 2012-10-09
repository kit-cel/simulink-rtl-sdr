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
   Interface betweeen rtl-sdr and MATLAB.
*/

/* Simulink */
#include "mex.h"

/* misc */
#include "utilities.h"
#include <string.h>

/* rtl-sdr includes */
#include <rtl-sdr.h>

/* interface */
#define DEVICE_INDEX  prhs[0]
#define SAMPLE_RATE   prhs[1]
#define FREQUENCY     prhs[2]
#define GAIN          prhs[3]
#define AGC_ON        prhs[4]
#define BUF_LENGTH    prhs[5]
#define RECEIVE_DATA  plhs[0]
#define num_inputs    6
#define num_outputs   1

/* defines */
#define NUM_SUPPORT	  20
#define TUNE_AUTO     0
#define TUNE_MANUAL   1
#define MAX_NO_GAINS  100

/* global variables */
rtlsdr_dev_t *_devices [NUM_SUPPORT];
int _sample_rates      [NUM_SUPPORT];
int _frequencies       [NUM_SUPPORT];
int _gains             [NUM_SUPPORT];
int _agcs_on           [NUM_SUPPORT];
int _buf_lengths       [NUM_SUPPORT];

/* Callback Data for asynchron read */
struct CallbackData {

    int      device_index;
    uint8_t* buffer;
    int      buf_length;
};

uint8_t bcnt, uninit = 1;

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	CallbackData *cb_data = (CallbackData*)ctx;

	if (ctx) {
		/*(*cb_data).buffer = (unsigned char*) buf;*/
		memcpy((*cb_data).buffer,(unsigned char*) buf,((*cb_data).buf_length<<1));
		rtlsdr_cancel_async(_devices[(*cb_data).device_index]);
	}
}


/* helper function */
int get_nearest_tuner_gain(const int* gains, int gain)
{
    for (int i = 0; i<MAX_NO_GAINS-1; i++)
        if ( gain <= gains[i] || gains[i] > gains[i+1] ) return gains[i];
    // should never happen
    return gains[0];
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	int ret;
	char errmsg[250];

    if (nlhs == 0 && nrhs == 1) {

    	/* check input */
    	CHECK_DEVICE_INDEX(DEVICE_INDEX);

    	/* get device index from input */
    	int device_index = (int)mxGetScalar(DEVICE_INDEX);

    	/* reset settings */
	    _sample_rates[device_index] = -1;
		_frequencies [device_index] = -1;
		_gains       [device_index] = -1000;
		_agcs_on     [device_index] = -1;
		_buf_lengths [device_index] = -1;

    	/* check if device is used */
    	if (_devices[device_index]) {

    		/* close device */
    		ret = rtlsdr_close(_devices[device_index]);
    		if (ret < 0) {

		    	sprintf(errmsg,"Failed to close rtl-sdr device #%d.\n",device_index); mexErrMsgTxt(errmsg);
		        return;
		    }

        	_devices     [device_index] = NULL;

		    mexPrintf("Closed rtl-sdr device #%d.\n",device_index);
		}
		/* device is not used */
		else {

			/* open rtl-sdr device */
		    ret = rtlsdr_open(&(_devices[device_index]), device_index);
		    if (ret < 0) {

		    	sprintf(errmsg,"Failed to use rtl-sdr device #%d.\n",device_index); mexErrMsgTxt(errmsg);
		        return;
		    }

		    /* reset endpoint before we start reading from it (mandatory) */
		    ret = rtlsdr_reset_buffer(_devices[device_index]);
		    if (ret < 0) {
		        mexErrMsgTxt("Failed to reset buffer.\n");
		    }

		    mexPrintf("Using rtl-sdr device #%d: %s\n",device_index,rtlsdr_get_device_name(device_index));
		}
    }    
    else if (nrhs == num_inputs && nlhs == num_outputs) {

    	/* check input */
    	CHECK_DEVICE_INDEX(DEVICE_INDEX);
    	CHECK_SAMPLE_RATE (SAMPLE_RATE );
	    CHECK_FREQUENCY   (FREQUENCY   );
	    CHECK_GAIN        (GAIN        );
	    CHECK_AGC_ON      (AGC_ON      );
	    CHECK_BUF_LENGTH  (BUF_LENGTH  );

    	/* get device index from input */
    	int device_index = (int)mxGetScalar(DEVICE_INDEX);

    	rtlsdr_dev_t *_device = _devices[device_index];

	    /* check if device is already in use */
    	if (!_devices[device_index]) {

    		 sprintf(errmsg,"rtl-sdr device #%d is not initialized.\n",device_index); mexErrMsgTxt(errmsg);
		}

		/* set sample rate */
	    int sample_rate = (int)mxGetScalar(SAMPLE_RATE);
	    if (sample_rate != _sample_rates[device_index]) {

	    	ret = rtlsdr_set_sample_rate(_device, (uint32_t)sample_rate);
		    if (ret < 0) {
		        sprintf(errmsg,"Failed to set sample rate to %u Sps.\n",sample_rate); mexErrMsgTxt(errmsg);
		    }

	    	_sample_rates[device_index]=sample_rate;
		}

		/* set tuning frequency */
		int frequency = (int)mxGetScalar(FREQUENCY);
	    if (frequency != _frequencies[device_index]) {

	    	ret = rtlsdr_set_center_freq(_device, (uint32_t)frequency);
	        if (ret < 0) {
	            sprintf(errmsg,"Failed to set center frequency to %u Hz.\n",frequency); mexErrMsgTxt(errmsg);
	        }

	        _frequencies[device_index] = frequency;
	    }

	    /* set gain */
	    int gain = (int)(10*(double)mxGetScalar(GAIN));
	    if (gain != _gains[device_index]) {

	    	if (gain == 0) {

	    		/* enable automatic gain */
	            ret = rtlsdr_set_tuner_gain_mode(_device, TUNE_AUTO);
	            if (ret < 0) {
	                sprintf(errmsg,"Failed to enable automatic gain.\n"); mexErrMsgTxt(errmsg);
	            }
	    	}
	    	else {
	            /* enable manual gain */
	            ret = rtlsdr_set_tuner_gain_mode(_device, TUNE_MANUAL);
	            if (ret < 0) {
	                sprintf(errmsg,"Failed to enable manual gain.\n"); mexErrMsgTxt(errmsg);
	            }

	            /* get gain list */
			    int* gains = (int*)malloc(MAX_NO_GAINS);
			    ret = rtlsdr_get_tuner_gains(_device, gains);
			    if (ret <= 0) {
			        sprintf(errmsg,"Failed to get rtl-sdr Tuner gains.\n"); mexErrMsgTxt(errmsg);
			    }
	            
	            /* set the tuner gain */
	            int actual_gain = get_nearest_tuner_gain(gains,gain);
	            ret = rtlsdr_set_tuner_gain(_device, actual_gain);
	            if (ret < 0) {
	                sprintf(errmsg,"Failed to set Tuner gain to %0.1f dB.\n", (double)gain/10.0); mexErrMsgTxt(errmsg);
	            }
	            else {
	                if (actual_gain != gain )
	                    mexPrintf("A gain of %0.1f dB is not supported.\n", (double)gain/10.0);
	                	mexPrintf("Tuner gain set to %0.1f dB.\n", (double)actual_gain/10.0);
	            }
	        }

	        _gains[device_index] = gain;
	    }

	    /* enable or disable the internal digital AGC */
	    int agc_on = (int)mxGetScalar(AGC_ON);
	    if (agc_on != _agcs_on[device_index]) {	

		    ret = rtlsdr_set_agc_mode(_device, agc_on);
		    if (ret < 0) {
		         sprintf(errmsg,"Failed to set AGC mode.\n"); mexErrMsgTxt(errmsg);
		    }

		    _agcs_on[device_index] = agc_on;
		}

		/* set buffer length */
	    int buf_length = (int)mxGetScalar(BUF_LENGTH);
	    if (buf_length != _buf_lengths[device_index]) {
	
			_buf_lengths[device_index] = buf_length;
		}

		/* allocate buffer memory */
		uint8_t *buffer = (uint8_t*) malloc((buf_length<<1) * sizeof(uint8_t));
		int n_read;

		/* pass data to Callback Data structure */
		CallbackData cb_data;
		cb_data.device_index = device_index;
		cb_data.buffer       = buffer;
		cb_data.buf_length   = buf_length;

		/* read data asynchron from device */
		ret = rtlsdr_read_async(_device,
						  		rtlsdr_callback,
						  		(void*)&cb_data,
						  		0,
						  		(buf_length<<1));
	  	if (ret < 0) {
	  		sprintf(errmsg,"Failed to read from device.\n"); mexErrMsgTxt(errmsg);
    	}	

	    /* create output data */
		RECEIVE_DATA = mxCreateDoubleMatrix(buf_length, 1, mxCOMPLEX);
    	double *outr = (double*)mxGetPr(RECEIVE_DATA);
    	double *outi = (double*)mxGetPi(RECEIVE_DATA);

		int k;
		/* pass buffer values to output */
	    for (k=0; k<buf_length; k++) {

	        /* scaling */
	        outr[k] = (((double)(buffer[ (k<<1)   ])) - 127.5f)/128.0f;
	        outi[k] = (((double)(buffer[((k<<1)+1)])) - 127.5f)/128.0f;
	    }

	    /* free the allocated buffer memory */
	    free((void*)buffer);
    }
    else {

    	/* Usage */
    	mexPrintf("\nUsage:"
    				"\n\n\t# Initialize/Close rtl-sdr device:\n\n"
    				  "     \t\trtlsdr_dev(index)\n\n"
    				  "     \t# Receive IQ-data from rtl-sdr device:\n\n"
                      "     \t\tdata = rtlsdr_dev(index,samplerate,frequency,gain,agc,buf_length)\n\n\n"
                      "     \t      index - The device index (e.g. 0).\n"
                      "     \t samplerate - The sampling rate of the device (e.g. 1e6 for 1 MHz bandwidth).\n"
                      "     \t  frequency - The center frequency of the tuner (e.g. 100e6 for 100 MHz).\n"
                      "     \t       gain - The overall gain of the receiver. Use 0 for automatic.\n"
                      "     \t        agc - Switch the internal digital AGC On or Off (0 means Off).\n"
                      "     \t buf_length - The number of samples in the receive buffer (e.g. 1000).\n"
                      "     \t       data - The received IQ-data.\n\n");

    	return;
    }
}