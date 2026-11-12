// Adt.h
// Bell Electrolabs Automatic Double Tracker, modelled from Krystal's trace
// (Docs/Bell_ADT_schematic.pdf):
//
//   input -> C1 30n (R1 470K) -> R2 1K -> U2 LM741 non-inverting:
//      GAIN (A100K wired as a variable resistor) from the output to the
//      inverting input, R4 1K + C3 10u to ground: x1 .. x101
//   U2 output = the direct signal: MIX pin 1 (and the direct out jack)
//      -> R6 33K || C7 3n3 (pre-emphasis) -> C8 10u -> U4 NE571 channel 1
//         as a compressor (input on R3, G cell and rectifier fed back from
//         the output through C10 10u, C11 1u rectifier cap, C12 33p)
//      -> C13 100n / R13 47K -> two Sallen-Key low-pass filters on emitter
//         followers: R17 R18 33K, C16 || C17 660p, C20 220p (Q4) and
//         R25 R29 33K, C23 330p, C25 220p (Q7)
//      -> C28 10u -> R37 R39 47R -> U5 MN3005 (4096-stage BBD; input bias
//         from TRIM1, outputs summed through R42 / R45 15K and TRIM2)
//      -> C31 1n, R34 100K, C34 100n -> the same two filters again
//         (R43 R41 33K, C32 || C33 660p, C29 220p, Q8; R32 R31 33K,
//         C27 330p, C26 220p, Q6)
//      -> C24 10u -> U4 NE571 channel 2 as an expander (C19 1u), feedback
//         R21 47K || C21 3n3 (de-emphasis) in series with the internal R3,
//         C22 330p across R3, R24 47K for the DC bias
//      -> C18 10u -> MIX pin 3 (the double), and through R20 15K to the
//         level indicator (Q1-Q5, D1 D2, LED D3)
//   MIX (B100K) wiper -> C9 10u -> R8 15K (R9 470K) -> U3 LM741 follower
//      -> C5 10u -> mixed output
//   Clock: U7 CD4046 VCO, C38 220p, R1 = R55 4K7 + DELAY (A100K as a
//      variable resistor); VCOin from R51 270K to +15 and R50 100K from the
//      DEV wiper. FOUT drives the MN3005 through the CD4049 buffers.
//   Modulation: U6 LM741 with R46 330K / R52 150K hysteresis and C36 30n
//      from the output to the inverting input, timing network R47 2K7 +
//      MOD (A100K as a variable resistor) into C39 4u7, then R54 33K into
//      C37 10u across DEV (B100K).
//
// Modelled as a circuit: every resistor and capacitor in the audio path
// (each stage from its transfer function), the 741s and NE571s clipping at
// their rails (the NE571 outputs sit off-centre, so they clip unevenly), the
// NE571 gain cells driven by their own rectifiers, the MN3005 as a clocked
// sample-and-hold chain with its two outputs averaged, and the modulation
// oscillator simulated from its components so its rate, shape and depth
// come from MOD, DEV and the capacitors.
// Modelled behaviourally, values estimated (named constants in Adt.cpp):
// the 4046's frequency law, the MN3005's gain and clipping level, and the
// NE571's unity-gain level.
//
// Volts in, volts out (1.0 = 1 V). Runs at 4x with Oversampler4x.
// Plain C++98, no allocation after construction.

#ifndef TBADT_ADT_H
#define TBADT_ADT_H

#include "Filters.h"
#include "Oversampler4x.h"

namespace tbadt
{

class Adt
{
public:
    Adt();

    void prepare (double sampleRate);
    void reset();

    // Front panel (0..1 pot rotation)
    void setGain (double v);            // A100K: U2's gain
    void setMix (double v);             // B100K: direct .. double
    void setMod (double v);             // A100K: modulation speed
    void setDev (double v);             // B100K: modulation depth
    void setDelay (double v);           // A100K: the 4046's timing resistor
    void setAdtOn (bool on);            // ADT in / out
    // Plugin only
    void setInputGainDb (double dB);
    void setOutputGainDb (double dB);

    // Mono in, mono out
    void process (const float* in, float* out, int numSamples);

    static int latencySamples() { return Oversampler4x::latencySamples(); }

    // Introspection for the tests and the panel
    static double audioTaper (double x);
    static double gainFactor (double gainPot);              // U2's mid-band gain
    static double clockHz (double delayPot, double vcoIn);  // 4046 VCO
    static double restVcoIn();                              // VCOin with no modulation
    static double delayMs (double delayPot);                // at rest
    double currentDelayMs() const { return 1000.0 * curDelay; }
    double lfoOutput() const { return lfoO; }               // U6 output, volts
    double vcoInput() const { return vcoIn; }
    double peakLight() const { return ledLevel; }           // D3: 0..1
    double peakBbd() const { return peakBbdLevel; }
    void   clearPeakBbd() { peakBbdLevel = 0.0; }
    double compressorEnv() const { return envC; }
    double expanderEnv() const { return envE; }

    enum { kStages = 4096, kRing = 8192 };

private:
    double tickOs (double x);
    void   stepLfo (double dt);
    void   updateGain (double gainPot);
    void   updateMix (double mixPot);

    double fsBase, fsOs;
    Oversampler4x up, down;

    // Targets and smoothed values
    double gainT, mixT, modT, devT, delayT, adtT, inT, outT;
    double gainP, mixP, modP, devP, delayP, adt, inG, outG;
    double smKnob, smSwitch;
    double gainApplied, mixApplied;
    int countdown;

    // Input and U2
    FirstOrder c1Hp;            // C1 / R1
    FirstOrder u2Net;           // 1 + GAIN / (R4 + 1/sC3)
    FirstOrder u2Gbw;           // the 741's bandwidth at that gain
    Slew u2Slew;
    // Compressor U4 ch.1
    FirstOrder preEmph;         // R6 || C7 in series with R3
    FirstOrder c8Hp;
    double vComp, iPrev, envC;
    double kEnvC, kEnvE;
    // Filters into the BBD
    FirstOrder c13Hp;
    Biquad sk1, sk2;
    // MN3005
    float ring[kRing];
    int ringPos;
    double tickPhase, bbdHeld, lastBbdIn, curDelay;
    FirstOrder c31Lp, c34Hp;
    Biquad sk3, sk4;
    FirstOrder c24Hp;
    // Expander U4 ch.2
    double envE;
    Biquad expFb;               // (R21 || C21 + R3 || C22) / R3
    FirstOrder c18Hp;
    // Mix and output
    double mixWet, mixDry;
    FirstOrder c9Hp, c5Hp;
    // Level indicator D3
    double ledLevel, kLedDecay;

    // Modulation oscillator (U6) and the 4046's control voltage
    double lfoO, lfoVm, lfoX, lfoY;
    bool lfoLinear;
    double vcoIn;
    int lfoSub;

    double peakBbdLevel;
};

} // namespace tbadt

#endif
