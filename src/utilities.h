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
    Header file for rtlsdr_source
*/

#ifndef UTILITIES
#define UTILITIES

/* printf for "ssWarning" */
static char errorMessage[512];
#define ssSetErrorStatusf(S, message, ...) \
    sprintf(errorMessage, message, __VA_ARGS__); \
    ssSetErrorStatus(S, errorMessage);

#define ssWarningf(S, message, ...) \
    {char errMsg[512];sprintf(errMsg, message, __VA_ARGS__);ssWarning(S, errMsg);}

#define NUMERIC_OR_DIE(S, param) \
    if (!mxIsNumeric( ssGetSFcnParam(S,param) )) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be numeric", "");return;}

#define NUMERIC_NOTEMPTY_OR_DIE(S, param) \
    if (!mxIsNumeric( ssGetSFcnParam(S,param) ) || mxIsEmpty( ssGetSFcnParam(S,param) ) ) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be numeric", "");return;}

#define CHECK_DEVICE_INDEX(param) \
if (!mxIsNumeric(param) || (mxGetScalar(param) < 0)) \
    {mexErrMsgTxt("Device Index must be numeric");return;}

#define CHECK_DEVICE_HANDLE(param) \
char tmp[100]; sprintf(tmp,"%d",*(int**)mxGetPr(param)); \
if (!mxIsDouble(param) || mxIsComplex(param) || !(mxGetN(param)==1) || !(mxGetM(param)==1) || mxIsEmpty(param) || !atoi(tmp)) \
    {mexErrMsgTxt("Parameter to mex-function must be numeric");return;}

#define CHECK_SAMPLE_RATE(param) \
if (!mxIsNumeric(param) || mxIsEmpty(param) || !(mxGetScalar(param) > 0)) \
    {mexErrMsgTxt("Samples per Second must be numeric and greater zero");return;}

#define CHECK_FREQUENCY(param) \
if (!mxIsNumeric(param) || !(mxGetScalar(param) > 0)) \
    {mexErrMsgTxt("Frequency must be numeric and greater zero");return;}

#define CHECK_GAIN(param) \
if (!mxIsNumeric(param)) \
    {mexErrMsgTxt("Gain must be numeric");return;}

#define CHECK_AGC_ON(param) \
if (!mxIsNumeric(param) || mxIsEmpty(param)) \
    {mexErrMsgTxt("Autmatic Gain Control must be numeric");return;}

#define CHECK_BUF_LENGTH(param) \
if (!mxIsNumeric(param) || mxIsEmpty(param) || !(mxGetScalar(param) > 0)) \
    {mexErrMsgTxt("Buffer Length must be numeric and greater zero");return;}

#endif /* UTILITIES */