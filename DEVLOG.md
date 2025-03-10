# DEVLOG

## System Description

The Light Controller (LC) is a device that converts analog (audio) signals into outputs that drive 12V RGB led strips.
LC will have 8 analog inputs and 8 led strip outputs. At a later stage a DMX input/output will be added to increase the
flexibility of the device.

-

## 10-03-2025

Calculating the transient from an audio waveform is quite slow.
For 8 channels and a block size of 16 it takes approximately 750us.
Marginal gains to the speed were reached by re-implementing the algorithm
for this specific use case.

- Move the two envelope generators to an analog circuit and measure the
  "fast" envelope for the dynamics and the "fast" and "slow" envelopes
  differentially to get the transient.
- Stick with the current software solution but use a "low" sample rate.
