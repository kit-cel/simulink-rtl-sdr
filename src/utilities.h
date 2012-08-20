/*
* Copyright 2010 Communications Engineering Lab, KIT
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

#endif /* UTILITIES */