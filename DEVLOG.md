# DEVLOG

## System Description

The Light Controller (LC) is a device that converts analog (audio) signals into outputs that drive 12V RGB led strips.
LC will have 8 analog inputs and 8 led strip outputs. At a later stage a DMX input/output will be added to increase the
flexibility of the device.

## Key Decisions

[KD1]: What analog/digital signals to use as the input.
For development simplicity sake "audio" input was chosen as the desired input signal

- [x] 1. Audio input
- [ ] 2. Gate input
- [ ] 3. Envelope input
- [ ] 4. Dual envelope input (differential)

[KD2]: How to drive the 12V LED outputs
For this early prototype stage the cheapness and simplicity of the P9813 modules was chosen as the best solution

- [ ] Custom solution
- [x] P9813 Modules
- [ ] TLC5947

[KD3]:

## Knowledge Gaps

[KG1] How best to sample the drum hits?
By experimentation I determined that the cheap piezo pick-ups I had laying
around are adequate for sampling drums.

The best results thus far were reached by dampening them with a piece of
anti-static foam, wrapping them in electrical tape and sticking them to
the bottom resonator of a tom with double-sided tape.

## 10-03-2025

Calculating the transient from an audio waveform is quite slow.
For 8 channels and a block size of 16 it takes approximately 750us.
Marginal gains to the speed were reached by re-implementing the algorithm
for this specific use case.

Perhaps a different route should be taken?

- Move the two envelope generators to an analog circuit and measure the
  "fast" envelope for the dynamics and the "fast" and "slow" envelopes
  differentially to get the transient.
- Stick with the current software solution but use a "low" sample rate.

When testing the software implementation on a tom with a rudimentary pre-amp
and conditioning stage the results were satisfactory.

## 11-03-2025

A very nice to have features would be detection of the dynamics of a hit.
However this represents several challenges:

- With the current Transient Detection algorithm only a "unit" trigger is generated.
- RMS calculations are fairly expensive although they could be delayed to when they are necessary
