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
    Interface betweeen rtl-sdr and Simulink.
*/

#define S_FUNCTION_NAME  rtlsdr_source
#define S_FUNCTION_LEVEL 2

/* Simulink */
#include "simstruc.h"

/* misc */
#include "utilities.h"

/* rtl-sdr includes */
#include <rtl-sdr.h>

/* boost */
//#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/condition_variable.hpp>
#include <pthread.h>

/* defines */
#define TUNE_AUTO                0
#define TUNE_MANUAL              1
#define MAX_NO_GAINS           100

#define BUF_SIZE  (16 * 32 * 512)
#define BUF_NUM   32
#define BUF_SKIP  1 // buffers to skip due to initial garbage
#define BYTES_PER_SAMPLE  2 // rtl device delivers 8 bit unsigned IQ data

struct SampleBuffer
{
    unsigned char **buf;
    unsigned int  num;
    unsigned int  head;
    unsigned int  used;

    unsigned int offset;
    int samp_avail;

    unsigned int skipped;

    bool overrun;
    bool overran_before;
};

/* S-function params */
enum SFcnParamsIndex
{
    DEVICE_INDEX=0,
    SAMPLE_RATE,
    FREQUENCY,
    GAIN,
    AGC_ON,
    FRAME_LENGTH,
    USE_FRAMES,

    /* NUM_PARAMS must be the last in this enum to correctly set the number
     * of expected parameters.
     */
    NUM_PARAMS
};

enum PWorkIndex
{
    DEVICE,   /* rtl-sdr object */
    GAINS,    /* list of possible gain values */

    THREAD,   /* boost thread object for reading samples from dev */
    MUTEX,    /* manages access to SBUF */
    COND_VAR,
    SBUF,     /* sample buffer struct */

    P_WORK_LENGTH
};

enum IWorkIndex
{
    FREQUENCY_PORT_INDEX, /* port index of FREQUENCY signal, 0 if none */
    GAIN_PORT_INDEX,      /* port index of GAIN signal, 0 if none */

    I_WORK_LENGTH
};

enum RWorkIndex
{
    LAST_FREQUENCY, /* holds current FREQUENCY (for port based setting) */
    LAST_GAIN,      /* holds current GAIN (for port based setting) */

    R_WORK_LENGTH
};


/* ======================================================================== */
#if defined(MATLAB_MEX_FILE)
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
/* ======================================================================== */
{
    NUMERIC_NOTEMPTY_OR_DIE(S,DEVICE_INDEX);
    NUMERIC_NOTEMPTY_OR_DIE(S,SAMPLE_RATE);
    NUMERIC_OR_DIE(S,FREQUENCY);
    NUMERIC_OR_DIE(S,GAIN);
    NUMERIC_NOTEMPTY_OR_DIE(S,AGC_ON);
    NUMERIC_NOTEMPTY_OR_DIE(S,FRAME_LENGTH);
    NUMERIC_NOTEMPTY_OR_DIE(S,USE_FRAMES);
}
#endif /* MDL_CHECK_PARAMETERS */


/* ======================================================================== */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
/* ======================================================================== */
{
    int_T port;

    /* set number of expected parameters and check for a mismatch. */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
         return;
    }
    #endif

    /* sampling */
    ssSetNumSampleTimes(S, PORT_BASED_SAMPLE_TIMES);

    /* Set number of input ports and tunability */
    ssSetSFcnParamTunable(S, DEVICE_INDEX, SS_PRM_NOT_TUNABLE);

    /* check whether frequency or gain is tunable */
    int_T num_input_ports = 0;
    if (mxIsEmpty(ssGetSFcnParam(S, FREQUENCY))) {
        ssSetSFcnParamTunable(S, FREQUENCY, SS_PRM_NOT_TUNABLE);
        num_input_ports++;
    }

    if (mxIsEmpty(ssGetSFcnParam(S, GAIN))) {
        ssSetSFcnParamTunable(S, GAIN, SS_PRM_NOT_TUNABLE);
        num_input_ports++;
    }

    /* set the resulting number of ports */
    if (!ssSetNumInputPorts(S, num_input_ports)) return;

    /* port properties */
    for(port = 0; port<num_input_ports; port++)
    {
        /* RF_FREQ and GAIN both have the same port spec */
        ssSetInputPortMatrixDimensions (S, port, 1, 1);
        ssSetInputPortComplexSignal    (S, port, COMPLEX_NO);
        ssSetInputPortDataType         (S, port, SS_DOUBLE);
        ssSetInputPortFrameData        (S, port, FRAME_INHERITED);
        ssSetInputPortDirectFeedThrough(S, port, 1);
        ssSetInputPortSampleTime       (S, port, INHERITED_SAMPLE_TIME);
        ssSetInputPortOffsetTime       (S, port, 0.0);
    }

    /* Set number of output ports */
    if (!ssSetNumOutputPorts(S, 1)) return;
    /* data port properties */
    port = 0;
    {
        double sample_time = 1/mxGetScalar(ssGetSFcnParam(S, SAMPLE_RATE));

        /* get data port properties */
        const int_T buf_length      = (int_T) (double)mxGetScalar(ssGetSFcnParam(S, FRAME_LENGTH));
        const Frame_T outputsFrames = (       (double)mxGetScalar(ssGetSFcnParam(S, USE_FRAMES))>0.0)? FRAME_YES : FRAME_NO;
        const time_T period         = (time_T)(sample_time * buf_length);

        /* set data port properties */
        ssSetOutputPortMatrixDimensions(S, port, buf_length, 1);
        ssSetOutputPortComplexSignal   (S, port, COMPLEX_YES);
        ssSetOutputPortDataType        (S, port, SS_DOUBLE);
        ssSetOutputPortFrameData       (S, port, outputsFrames);
        ssSetOutputPortSampleTime      (S, port, period);
        ssSetOutputPortOffsetTime      (S, port, 0.0);
    }

    /* Prepare work Vectors */
    ssSetNumPWork(S, P_WORK_LENGTH);
    ssSetNumIWork(S, I_WORK_LENGTH);
    ssSetNumRWork(S, R_WORK_LENGTH);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

/* ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
/* ======================================================================== */
{
    /* PORT_BASED_SAMPLE_TIMES */
}

/* ======================================================================== */
/* HELPER FUNCTION */
int get_nearest_tuner_gain(const int* gains, int gain)
/* ======================================================================== */
{
    for (int i = 0; i<MAX_NO_GAINS-1; i++)
        if ( gain <= gains[i] || gains[i] > gains[i+1] ) return gains[i];
    // should never happen
    return gains[0];
}


/* ======================================================================== */
static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
/* ======================================================================== */
{
    // get access to Simulink stuff
    SimStruct *S = (SimStruct *)ctx;
    SampleBuffer *sbuf = (SampleBuffer *)ssGetPWorkValue(S, SBUF);

    // skip first few buffers
    if (sbuf->skipped < BUF_SKIP) {
        sbuf->skipped++;
        return;
    }

    // write received samples to sample buffer
    pthread_mutex_t* mutex = (pthread_mutex_t*)ssGetPWorkValue(S, MUTEX );
    pthread_mutex_lock( mutex );

    // get pos write pos in buffers
    int buf_tail = (sbuf->head + sbuf->used) % sbuf->num;
    // copy samples to sample buffer
    memcpy(sbuf->buf[buf_tail], buf, len);

    // check for overruns
    if (sbuf->used == sbuf->num) {
        sbuf->overrun = true;
        sbuf->overran_before = true;
        sbuf->head = (sbuf->head + 1) % sbuf->num;
    } else {
        sbuf->used++;
    }

    pthread_mutex_unlock( mutex );

    // notify output function
    pthread_cond_signal( (pthread_cond_t*)ssGetPWorkValue(S, COND_VAR ) );
}

/* ======================================================================== */
static void * rtlsdr_wait(void *ctx)
/* ======================================================================== */
{
    // get access to Simulink stuff
    SimStruct *S = (SimStruct *)ctx;

    // get device handle
    rtlsdr_dev_t *_device = (rtlsdr_dev_t*)ssGetPWorkValue(S, DEVICE);
    // request samples
    int ret = rtlsdr_read_async( _device, rtlsdr_callback, (void *)S, 0, BUF_SIZE );
    // (on terminate)
    if ( ret != 0 ) {
        ssSetErrorStatusf(S,"rtlsdr_read_async returned with %d.\n", ret);
    }

    return NULL;
}


/* ======================================================================== */
#define MDL_START
static void mdlStart(SimStruct *S)
/* ======================================================================== */
{
    int ret = -1;
    const uint32_t device_index = (uint32_t)mxGetScalar(ssGetSFcnParam(S, DEVICE_INDEX));
    const double   sample_rate  =           mxGetScalar(ssGetSFcnParam(S, SAMPLE_RATE ));

    /* Set options of this Block */
    ssSetOptions(S, ssGetOptions(S) | SS_OPTION_CALL_TERMINATE_ON_EXIT);

    /* allocate memory for rtl-sdr object */
    rtlsdr_dev_t *_device = (rtlsdr_dev_t*)malloc(8);

    /* allocate memory for sample buffer */
    SampleBuffer *sbuf = (SampleBuffer*)malloc( sizeof(SampleBuffer) );
    sbuf->num = BUF_NUM;
    sbuf->head = sbuf->used = sbuf->offset = sbuf->skipped = 0;
    sbuf->samp_avail = BUF_SIZE / BYTES_PER_SAMPLE;
    sbuf->overrun = sbuf->overran_before = false;
    sbuf->buf = (unsigned char **) malloc( sbuf->num * sizeof(unsigned char *));
    if (sbuf->buf) {
        for(unsigned int i = 0; i < sbuf->num; ++i)
            sbuf->buf[i] = (unsigned char *) malloc( BUF_SIZE * sizeof(unsigned char));
    }
    ssSetPWorkValue(S, SBUF, sbuf);

    /* check for optional ports */
    int next_port = 0; /* no fixed/other inputs */
    if( mxIsEmpty(ssGetSFcnParam(S,FREQUENCY)) )
        ssSetIWorkValue(S, FREQUENCY_PORT_INDEX, next_port++);
    if( mxIsEmpty(ssGetSFcnParam(S, GAIN)) )
        ssSetIWorkValue(S, GAIN_PORT_INDEX, next_port);

    /* set impossible values as last values */
    ssSetRWorkValue(S, LAST_FREQUENCY, -1  );
    ssSetRWorkValue(S, LAST_GAIN,      -1e7);

    /* give handle to PWork vector */
    ssSetPWorkValue(S, DEVICE, NULL);

    /* open rtl-sdr device */
    ret = rtlsdr_open(&_device, device_index);
    if (ret < 0) {
        ssSetErrorStatusf(S,"Failed to open rtl-sdr device #%d", device_index);
        return;
    }

    /* give handle to PWork vector */
    ssSetPWorkValue(S, DEVICE, (rtlsdr_dev_t*)_device);

    /* get gain list */
    int* gains = (int*) malloc( MAX_NO_GAINS * sizeof(int));
    ret = rtlsdr_get_tuner_gains(_device, gains);
    if (ret <= 0) {
        ssSetErrorStatus(S, "Failed to get rtl-sdr Tuner gains.");
    }
    ssSetPWorkValue(S, GAINS, gains);

    /* show device name */
    ssPrintf("Using rtl-sdr device #%d: %s\n",device_index,rtlsdr_get_device_name(device_index),device_index);

    /* set sample rate */
    ret = rtlsdr_set_sample_rate(_device, (uint32_t)sample_rate);
    if (ret < 0) {
        ssSetErrorStatusf(S,"Failed to set sample rate to %u Sps.\n",(uint32_t)sample_rate);
    }
    ssPrintf("Sampling at %u Sps.\n", (uint32_t)sample_rate);

    /* set tuning frequency */
    if (mxIsEmpty(ssGetSFcnParam(S,FREQUENCY)) == 0) {
        const uint32_t frequency  = (uint32_t)mxGetScalar(ssGetSFcnParam(S, FREQUENCY));
        if (frequency>=0) {
            ret = rtlsdr_set_center_freq(_device, frequency);
            if (ret < 0) {
                ssSetErrorStatusf(S,"Failed to set center frequency to %u Hz.\n",frequency);
            }

            ssPrintf("Tuned to %u Hz.\n", frequency);
        } else {
            rtlsdr_set_testmode( _device, 1);
            ssPrintf("Testmode enabled");
        }
    }

    /* set gain */
    if (mxIsEmpty(ssGetSFcnParam(S,GAIN)) == 0) {
        int gain = (int)mxGetScalar(ssGetSFcnParam(S, GAIN));
        if (gain == 0) {

            /* Enable automatic gain */
            ret = rtlsdr_set_tuner_gain_mode(_device, TUNE_AUTO);
            if (ret < 0) {
                ssSetErrorStatus(S,"Failed to enable automatic gain.");
            }
            ssPrintf("Tuner gain set to automatic.\n");
        }
        else {
            /* Enable manual gain */
            ret = rtlsdr_set_tuner_gain_mode(_device, TUNE_MANUAL);
            if (ret < 0) {
                ssSetErrorStatus(S,"Failed to enable manual gain.");
            }

            /* Set the tuner gain */
            int actual_gain = get_nearest_tuner_gain((const int*)ssGetPWorkValue(S, GAINS),gain);
            ret = rtlsdr_set_tuner_gain(_device, actual_gain);
            if (ret < 0) {
                ssSetErrorStatusf(S, "Failed to set Tuner gain to %0.1f dB.", (double)gain/10.0);
            }
            else {
                if (actual_gain != gain )
                    ssPrintf("A gain of %0.1f dB is not supported\n", (double)gain/10.0);
                ssPrintf("Tuner gain set to %0.1f dB.\n", (double)actual_gain/10.0);
            }
        }
    }
    else {
        /* Enable manual gain for dynamic change*/
        ret = rtlsdr_set_tuner_gain_mode(_device, TUNE_MANUAL);
        if (ret < 0) {
            ssSetErrorStatus(S,"Failed to enable manual gain.");
        }
    }

    /* Enable or disable the internal digital AGC */
    ret = rtlsdr_set_agc_mode(_device, (int)mxGetScalar(ssGetSFcnParam(S, AGC_ON)));
    if (ret < 0) {
        ssSetErrorStatus(S,"Failed to set AGC mode.\n");
    }

    /* Reset endpoint before we start reading from it (mandatory) */
    ret = rtlsdr_reset_buffer(_device);
    if (ret < 0) {
        ssSetErrorStatus(S,"Failed to reset buffers.\n");
    }

    /* create mutex for sample thread */
    pthread_mutex_t *mutex = (pthread_mutex_t *) malloc(sizeof(pthread_mutex_t));
    //*mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_init(mutex,NULL);
    ssSetPWorkValue(S, MUTEX, mutex);

    /* create condition variable for sample thread */
    pthread_cond_t *cond_var = (pthread_cond_t *) malloc(sizeof(pthread_cond_t));
    pthread_cond_init(cond_var,NULL);
    ssSetPWorkValue(S, COND_VAR, cond_var);

    /* create thread for receiving samples from device */
    pthread_t *thread = (pthread_t *) malloc(sizeof(pthread_t));
    pthread_create( thread, NULL, rtlsdr_wait, S );
    ssSetPWorkValue(S, THREAD, thread);
}

/* ======================================================================== */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
/* ======================================================================== */
{
    const int_T frame_length = (int_T)(double)mxGetScalar(ssGetSFcnParam(S, FRAME_LENGTH));
    int   ret;

    /* get rtl-sdr object back from PWork vector */
    rtlsdr_dev_t *_device = (rtlsdr_dev_t*)ssGetPWorkValue(S, DEVICE);

    /* tune frequency if change occured */
    const int_T frequency_port       = mxIsEmpty(ssGetSFcnParam(S,FREQUENCY));
    const int_T frequency_port_index = ssGetIWorkValue(S, FREQUENCY_PORT_INDEX);
    if(frequency_port && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, frequency_port_index), tid) )
    {
        /* set new frequency */
        uint32_t new_frequency = (uint32_t)(*ssGetInputPortRealSignalPtrs(S,frequency_port_index)[0]);
        if(ssGetRWorkValue(S, LAST_FREQUENCY) != new_frequency)
        {
            ssSetRWorkValue(S, LAST_FREQUENCY, new_frequency);
            /* remember value to avoid unnecessary updates */
            /* set tuning frequency */
            ret = rtlsdr_set_center_freq(_device, new_frequency);
            if (ret < 0) {
                ssSetErrorStatusf(S,"Failed to set center frequency to %u Hz.\n",new_frequency);
            }
        }
    }

    /* change gain if change occured */
    const int_T gain_port       = mxIsEmpty(ssGetSFcnParam(S,GAIN));
    const int_T gain_port_index = ssGetIWorkValue(S, GAIN_PORT_INDEX);
    if(gain_port && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, gain_port_index), tid) )
    {
        /* set new gain */
        double new_gain = (double)(*ssGetInputPortRealSignalPtrs(S,gain_port_index)[0]);
        if(ssGetRWorkValue(S, LAST_GAIN) != new_gain)
        {
            ssSetRWorkValue(S, LAST_GAIN, new_gain);
            /* remember value to avoid unnecessary updates */
            /* Set the tuner gain */
            int actual_gain = get_nearest_tuner_gain((const int*)ssGetPWorkValue(S, GAINS),(int)(new_gain*10));
            ret = rtlsdr_set_tuner_gain((rtlsdr_dev_t*)ssGetPWorkValue(S, DEVICE), actual_gain);
            if (ret < 0) {
                ssSetErrorStatusf(S,"Failed to set tuner gain to %0.1f dB.\n", new_gain);
            }
        }
    }


    // get sample buffer work vars
    SampleBuffer *sbuf = (SampleBuffer *)ssGetPWorkValue(S, SBUF);
    pthread_mutex_t* mutex = (pthread_mutex_t*)ssGetPWorkValue(S, MUTEX );

    /* output buffer */
    double* out = (double*)ssGetOutputPortSignal(S, 0);

    pthread_mutex_lock( mutex );
    // wait for sample buffers to fill
    while (sbuf->used < 3) // collect at least 3 buffers
        pthread_cond_wait( (pthread_cond_t*)ssGetPWorkValue(S, COND_VAR ), mutex);

    // output overrun status (not in callback, due to issues with Simulink)
    if (sbuf->overrun) {
        ssPrintf("O");
        sbuf->overrun=false;
    }
    pthread_mutex_unlock( mutex );

    // get next samples to be read (select buffer + offset)
    unsigned char *buf = sbuf->buf[sbuf->head] + BYTES_PER_SAMPLE*(sbuf->offset);

    // enough samples in selected buffer for a whole frame?
    if (frame_length <= sbuf->samp_avail) {
        // read entire frame from current sample buffer
        for (int k = 0; k < BYTES_PER_SAMPLE*frame_length; ++k)
            out[k] = ((double)(buf[k]) - 127.5)/128.0;

        sbuf->offset += frame_length;
        sbuf->samp_avail -= frame_length;

    } else {
        // read as many as possible
        for (int k = 0; k < BYTES_PER_SAMPLE*sbuf->samp_avail; ++k)
            out[k] = ((double)(buf[k]) - 127.5)/128.0;

        // mark current buffer as used / empty
        pthread_mutex_lock( mutex );
        sbuf->head = (sbuf->head + 1) % sbuf->num;
        sbuf->used--;
        pthread_mutex_unlock( mutex );

        // set to start of the next sample buffer
        buf = sbuf->buf[sbuf->head];

        int remaining = frame_length - sbuf->samp_avail;

        // get the rest of samples
        for (int k = 0; k < BYTES_PER_SAMPLE*(remaining); ++k)
            out[k+BYTES_PER_SAMPLE*sbuf->samp_avail] = ((double)(buf[k]) - 127.5)/128.0;

        // update offset with the current buffer and the number of sample remaining
        sbuf->offset     = remaining;
        sbuf->samp_avail = (BUF_SIZE / BYTES_PER_SAMPLE) - remaining;
    }

}

/* ======================================================================== */
static void mdlTerminate(SimStruct *S)
/* ======================================================================== */
{
    /* check if rtl-sdr object has been created */
    if (ssGetPWorkValue(S, DEVICE))
    {
        rtlsdr_dev_t* _device = (rtlsdr_dev_t*)ssGetPWorkValue(S, DEVICE);

        rtlsdr_cancel_async( _device );
        pthread_join( * ((pthread_t*)ssGetPWorkValue(S, THREAD) ), NULL);
        rtlsdr_close( _device );

    }

    /* release memory for gain list */
    if (ssGetPWorkValue(S, GAINS))
        free(ssGetPWorkValue(S, GAINS));

    /* release thread stuff */
    if (ssGetPWorkValue(S, THREAD ))
        free( (pthread_t *) ssGetPWorkValue(S, THREAD ));
    if (ssGetPWorkValue(S, MUTEX )) {
        pthread_mutex_t *mutex = (pthread_mutex_t *) ssGetPWorkValue(S, MUTEX );
        pthread_mutex_destroy(mutex);
        free(mutex);
        mutex = NULL;
    }
    if (ssGetPWorkValue(S, COND_VAR )) {
        pthread_cond_t* cond_var = (pthread_cond_t *) ssGetPWorkValue(S, COND_VAR);
        pthread_cond_destroy(cond_var);
        free(cond_var);
        cond_var = NULL;
    }
    
    /* destroy sample buffer struct */
    if (ssGetPWorkValue(S, SBUF ))
    {
        SampleBuffer *sbuf = (SampleBuffer *)ssGetPWorkValue(S, SBUF);

        if (sbuf->overran_before) {
            ssPrintf("\n");
        }

        if (sbuf->buf) {
            for(unsigned int i = 0; i < sbuf->num; ++i) {
                if (sbuf->buf[i])
                    free(sbuf->buf[i]);
            }
            free(sbuf->buf);
        }
        free(sbuf);
    }
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
