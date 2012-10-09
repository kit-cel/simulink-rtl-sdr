%Usage:
%
%       # Initialize rtl-sdr device:
%
%           handle = rtlsdr_dev(index)
%
%       # Close rtl-sdr device:
%
%           rtlsdr_dev(handle)
%
%       # Receive IQ-samples from rtl-sdr device:
%
%           data = rtlsdr_dev(handle,samplerate,frequency,gain,agc,buf_length)
%
%
%             index - The device index (e.g. 0).
%            handle - The returned handle used for addressing the initialized rtl-sdr device.
%        samplerate - The sampling rate of the device (e.g. 1e6 for 1 MHz bandwidth)
%         frequency - The center frequency of the tuner (e.g. 100e6 for 100 MHz).
%              gain - The overall gain of the receiver. Use 0 for automatic.
%               agc - Switch the internal digital AGC On or Off (0 means Off).
%        buf_length - The number of samples in the receive buffer (e.g. 1000).
%              data - The received IQ-samples.
%