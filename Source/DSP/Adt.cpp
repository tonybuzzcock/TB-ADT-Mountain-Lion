// Adt.cpp
// See Adt.h for the signal path. Component values are from the drawing;
// the estimates (things the drawing doesn't fix) are marked.

#include "Adt.h"

#include <cmath>
#include <cstring>

namespace tbadt
{

namespace
{
    // Supplies: +15 V (Vb through R5 47R) and -12 V. The 741s swing to about
    // 2 V inside each rail.
    const double kOpHi = 13.0, kOpLo = -10.0;

    // Input: C1 30n into R1 470K; U2: R4 1K, C3 10u, GAIN 100K
    const double kC1 = 30.0e-9, kR1 = 470.0e3, kR4 = 1.0e3, kC3 = 10.0e-6, kGainPot = 100.0e3;
    const double kGbw = 1.0e6;                 // LM741 gain-bandwidth
    const double kSlew = 0.5e6;                // LM741 slew rate, V/s

    // NE571 internals (datasheet): R3 20K, rectifier 10K, unity-gain level
    // 0 dBu (0.775 V rms, 0.70 V average)
    const double kNe571R = 20.0e3, kNe571Rect = 10.0e3, kVu = 0.70;
    // Compressor U4 ch.1: R6 33K || C7 3n3 into R3; C8 10u; C11 1u; C12 33p
    const double kR6 = 33.0e3, kC7 = 3.3e-9, kC8 = 10.0e-6, kC11 = 1.0e-6, kC12 = 33.0e-12;
    // Expander U4 ch.2: C19 1u; R21 47K || C21 3n3; C22 330p across R3; C24 10u
    const double kC19 = 1.0e-6, kR21 = 47.0e3, kC21 = 3.3e-9, kC22 = 330.0e-12, kC24 = 10.0e-6;
    // NE571 output swing on the single +15 V supply. Each output sits at its
    // DC level, 1.8 V x (1 + feedback / input resistance): channel 1 at
    // 1.8 x (1 + (R10 + R11) / R3) = 7.7 V, channel 2 at
    // 1.8 x (1 + (R3 + R21) / R24) = 4.4 V; both swing from about 1 V to 13 V.
    const double kCompHi = 13.0 - 7.74, kCompLo = 7.74 - 1.0;
    const double kExpHi = 13.0 - 4.37, kExpLo = 4.37 - 1.0;

    // Filters: C13 100n into R13 47K; the Sallen-Keys (33K, 33K, 660p / 220p
    // and 33K, 33K, 330p / 220p) on BC184 emitter followers
    const double kC13 = 100.0e-9, kR13 = 47.0e3;
    const double kRsk = 33.0e3, kCfb1 = 660.0e-12, kCfb2 = 330.0e-12, kCgnd = 220.0e-12;
    const double kFollower = 0.99;
    // BBD outputs: R42 / R45 15K into the ends of TRIM2 22K (centred), so the
    // wiper averages the two outputs from about 13K; C31 1n and R34 100K load
    // it. C34 100n into R34: the drawing has R34 on the BBD side of C34, which
    // would leave Q8's base with no DC path, so it is taken as the bias for
    // the filter's input.
    const double kRtrimSrc = 0.5 * (15.0e3 + 11.0e3), kC31 = 1.0e-9, kR34 = 100.0e3, kC34 = 100.0e-9;
    // MIX B100K into C9 10u / R8 15K + R9 470K; C5 10u into R3 15K; C18 10u
    const double kMixPot = 100.0e3, kMixLoad = 15.0e3 + 470.0e3, kC9 = 10.0e-6, kC5 = 10.0e-6;
    const double kR3out = 15.0e3, kC18 = 10.0e-6;

    // Level indicator: the long-tailed pair Q1 / Q5 trips when the double
    // swings below about -2.1 V or above +5.1 V (from R12 R16 R23 R26, D1 D2
    // and R20); C14 100n recharging through R14 + R15 holds the LED on.
    const double kLedLo = -2.1, kLedHi = 5.1, kLedHold = (470.0e3 + 1.0e6) * 100.0e-9;

    // Modulation oscillator U6: R46 330K, R52 150K, C36 30n, R53 33K,
    // R47 2K7 + MOD 100K, C39 4u7, R54 33K, C37 10u, DEV 100K;
    // VCOin from R50 100K and R51 270K to +15 V
    const double kR46 = 330.0e3, kR52 = 150.0e3, kC36 = 30.0e-9, kR53 = 33.0e3;
    const double kR47 = 2.7e3, kModPot = 100.0e3, kC39 = 4.7e-6, kR54 = 33.0e3, kC37 = 10.0e-6;
    const double kDevPot = 100.0e3, kR50 = 100.0e3, kR51 = 270.0e3, kSupply = 15.0;
    const double kBeta = kR52 / (kR46 + kR52);

    // Clock: 4046 VCO, C1 = C38 220p, R1 = R55 4K7 + DELAY 100K, R2 open
    const double kC38 = 220.0e-12, kR55 = 4.7e3, kDelayPot = 100.0e3;

    // --- Estimates (not fixed by the drawing) ---------------------------------
    // 4046 VCO with R2 open: f = K (VCOin - Vt) / (R1 (C1 + 32p)). Vt is the
    // input follower's threshold; K is set so DELAY at noon gives about 30 ms
    // (10 ms fully anticlockwise, about 210 ms fully clockwise).
    const double kVcoVt = 1.0, kVcoK = 0.083, kVcoStray = 32.0e-12;
    // MN3005: gain through the chip and its clipping level at the input (TRIM1
    // centred)
    const double kBbdGain = 1.0, kBbdClip = 2.5;
    // Compressor's largest gain at very low levels (relative to R3)
    const double kCompMaxGain = 100.0;
    // -------------------------------------------------------------------------

    inline double clamp01 (double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

    // Output stage: linear inside its swing, rounding off into a hard limit
    // (hi above, lo below; both positive numbers).
    inline double swingClip (double v, double hi, double lo)
    {
        const double lim = v >= 0.0 ? hi : lo;
        const double soft = lim < 5.0 ? 0.4 * lim : 2.0;
        const double knee = lim - soft;
        const double a = std::fabs (v);
        if (a <= knee) return v;
        const double c = knee + soft * std::tanh ((a - knee) / soft);
        return v < 0.0 ? -c : c;
    }

    inline double softClip (double v, double limit)
    {
        const double knee = 0.7 * limit;
        const double a = std::fabs (v);
        if (a <= knee) return v;
        const double c = knee + (limit - knee) * std::tanh ((a - knee) / (limit - knee));
        return v < 0.0 ? -c : c;
    }

    inline double highpassFc (double r, double c) { return 1.0 / (2.0 * kPi * r * c); }

    // Unity-gain Sallen-Key low-pass on a follower of gain k: equal resistors
    // R, Cf from the middle node to the output, Cg from the input to ground.
    void setSallenKey (Biquad& b, double R, double Cf, double Cg, double k, double fs)
    {
        b.setAnalog (0.0, 0.0, k, R * R * Cf * Cg, 2.0 * R * Cg + R * Cf * (1.0 - k), 1.0, fs);
    }
}

//==============================================================================
double Adt::audioTaper (double x)
{
    const double k = 4.394449154672439;        // 10% of the track at mid-rotation
    if (x <= 0.0) return 0.0;
    if (x >= 1.0) return 1.0;
    return (std::exp (k * x) - 1.0) / (std::exp (k) - 1.0);
}

double Adt::gainFactor (double pot)
{
    return 1.0 + kGainPot * audioTaper (clamp01 (pot)) / kR4;
}

double Adt::clockHz (double pot, double vin)
{
    // DELAY clockwise puts more of the track in R1: a slower clock
    const double r1 = kR55 + kDelayPot * audioTaper (clamp01 (pot));
    double v = vin - kVcoVt;
    if (v < 0.05) v = 0.05;
    double f = kVcoK * v / (r1 * (kC38 + kVcoStray));
    if (f < 1000.0) f = 1000.0;
    return f;
}

double Adt::restVcoIn() { return kSupply * kR50 / (kR50 + kR51); }

// Two-phase clock at f, 4096 stages: N / (2 f)
double Adt::delayMs (double pot) { return 1000.0 * kStages / (2.0 * clockHz (pot, restVcoIn())); }

//==============================================================================
Adt::Adt()
    : fsBase (44100.0), fsOs (176400.0),
      gainT (0.0), mixT (0.5), modT (0.5), devT (0.5), delayT (0.5), adtT (1.0), inT (1.0), outT (1.0),
      gainApplied (-1.0), mixApplied (-1.0), countdown (0),
      vComp (0.0), iPrev (0.0), envC (0.0), kEnvC (0.0), kEnvE (0.0),
      ringPos (0), tickPhase (0.0), bbdHeld (0.0), lastBbdIn (0.0), curDelay (0.03),
      envE (0.0), mixWet (0.5), mixDry (0.5), ledLevel (0.0), kLedDecay (0.0),
      lfoO (kOpHi), lfoVm (0.0), lfoX (0.0), lfoY (0.0), lfoLinear (false), vcoIn (4.0), lfoSub (2),
      peakBbdLevel (0.0)
{
    std::memset (ring, 0, sizeof (ring));
    prepare (44100.0);
}

void Adt::prepare (double sampleRate)
{
    fsBase = sampleRate > 1000.0 ? sampleRate : 44100.0;
    fsOs = fsBase * Oversampler4x::kFactor;

    smKnob   = std::exp (-1.0 / (0.020 * fsBase));
    smSwitch = std::exp (-1.0 / (0.006 * fsBase));

    c1Hp.highpass (highpassFc (kR1, kC1), fsOs);
    u2Slew.step = kSlew / fsOs;

    preEmph.setAnalog (kNe571R * kR6 * kC7, kNe571R, kNe571R * kR6 * kC7, kR6 + kNe571R, fsOs);
    c8Hp.highpass (highpassFc (kR6 + kNe571R, kC8), fsOs);
    kEnvC = 1.0 - std::exp (-1.0 / (kNe571Rect * kC11 * fsOs));
    kEnvE = 1.0 - std::exp (-1.0 / (kNe571Rect * kC19 * fsOs));

    c13Hp.highpass (highpassFc (kR13, kC13), fsOs);
    setSallenKey (sk1, kRsk, kCfb1, kCgnd, kFollower, fsOs);
    setSallenKey (sk2, kRsk, kCfb2, kCgnd, kFollower, fsOs);

    {
        const double rs = kRtrimSrc * kR34 / (kRtrimSrc + kR34);
        c31Lp.lowpass (1.0 / (2.0 * kPi * rs * kC31), fsOs);
    }
    c34Hp.highpass (highpassFc (kR34, kC34), fsOs);
    setSallenKey (sk3, kRsk, kCfb1, kCgnd, kFollower, fsOs);
    setSallenKey (sk4, kRsk, kCfb2, kCgnd, kFollower, fsOs);
    c24Hp.highpass (highpassFc (kNe571R * kNe571Rect / (kNe571R + kNe571Rect), kC24), fsOs);

    // Expander feedback over R3: ((R21 || C21) + (R3 || C22)) / R3
    expFb.setAnalog (0.0,
                     kR21 * kNe571R * (kC22 + kC21), kR21 + kNe571R,
                     kNe571R * kR21 * kC21 * kNe571R * kC22,
                     kNe571R * (kR21 * kC21 + kNe571R * kC22), kNe571R, fsOs);
    c18Hp.highpass (highpassFc (kMixPot, kC18), fsOs);

    c9Hp.highpass (highpassFc (kMixLoad, kC9), fsOs);
    c5Hp.highpass (highpassFc (kR3out, kC5), fsOs);
    kLedDecay = std::exp (-1.0 / (kLedHold * fsOs));

    // The oscillator runs in steps of 20 us or less
    lfoSub = (int) std::ceil ((1.0 / fsBase) / 20.0e-6);
    if (lfoSub < 1) lfoSub = 1;

    reset();
}

void Adt::reset()
{
    gainP = gainT; mixP = mixT; modP = modT; devP = devT; delayP = delayT;
    adt = adtT; inG = inT; outG = outT;

    up.reset(); down.reset();
    c1Hp.reset(); u2Net.reset(); u2Gbw.reset(); u2Slew.reset();
    preEmph.reset(); c8Hp.reset();
    vComp = 0.0; iPrev = 0.0; envC = 0.0; envE = 0.0;
    c13Hp.reset(); sk1.reset(); sk2.reset();
    std::memset (ring, 0, sizeof (ring));
    ringPos = 0; tickPhase = 0.0; bbdHeld = 0.0; lastBbdIn = 0.0;
    c31Lp.reset(); c34Hp.reset(); sk3.reset(); sk4.reset(); c24Hp.reset();
    expFb.reset(); c18Hp.reset();
    c9Hp.reset(); c5Hp.reset();
    ledLevel = 0.0;
    peakBbdLevel = 0.0;

    gainApplied = mixApplied = -1.0;
    updateGain (gainP);
    updateMix (mixP);

    // Start the oscillator and let C37 / C39 settle (about 3 s of circuit time)
    lfoO = kOpHi; lfoVm = 0.0; lfoX = 0.0; lfoY = 0.0; lfoLinear = false;
    const double dt = 20.0e-6;
    for (int i = 0; i < 150000; ++i) stepLfo (dt);
    curDelay = kStages / (2.0 * clockHz (delayP, vcoIn));
    countdown = 0;
}

void Adt::setGain (double v)              { gainT = clamp01 (v); }
void Adt::setMix (double v)               { mixT = clamp01 (v); }
void Adt::setMod (double v)               { modT = clamp01 (v); }
void Adt::setDev (double v)               { devT = clamp01 (v); }
void Adt::setDelay (double v)             { delayT = clamp01 (v); }
void Adt::setAdtOn (bool on)              { adtT = on ? 1.0 : 0.0; }
void Adt::setInputGainDb (double dB)      { inT = std::pow (10.0, dB / 20.0); }
void Adt::setOutputGainDb (double dB)     { outT = std::pow (10.0, dB / 20.0); }

//==============================================================================
void Adt::updateGain (double pot)
{
    // U2 non-inverting: 1 + Rg / (R4 + 1 / s C3), Rg the GAIN track in circuit
    const double rg = kGainPot * audioTaper (pot);
    u2Net.setAnalog (kC3 * (kR4 + rg), 1.0, kR4 * kC3, 1.0, fsOs);
    // The 741 runs out of loop gain at GBW / noise gain
    double fc = kGbw / (1.0 + rg / kR4);
    if (fc > 0.45 * fsOs) fc = 0.45 * fsOs;
    u2Gbw.lowpass (fc, fsOs);
    gainApplied = pot;
}

void Adt::updateMix (double pot)
{
    // MIX wiper between the direct signal (pin 1) and the double (pin 3),
    // loaded by R8 + R9
    const double rs = pot * (1.0 - pot) * kMixPot;
    const double load = kMixLoad / (kMixLoad + rs);
    mixWet = pot * load;
    mixDry = (1.0 - pot) * load;
    mixApplied = pot;
}

// U6 relaxation oscillator and the network into the 4046's VCOin.
// Saturated, U6's inverting input follows C39 through R53 / C36; once it
// crosses the hysteresis level on the non-inverting input, U6 is in its
// linear region, where C36's feedback sets how fast it swings (positive
// feedback makes the swing run away to the other rail).
void Adt::stepLfo (double dt)
{
    const double rm = kR47 + kModPot * (1.0 - audioTaper (modP));     // wiper tied to pin 1
    double ra = kDevPot * (1.0 - devP), rb = kDevPot * devP;
    if (ra < 100.0) ra = 100.0;
    if (rb < 1.0) rb = 1.0;            // DEV fully anticlockwise: the wiper on ground
    const double rv = kR50 + kR51;

    const double vp = kBeta * lfoO;
    if (lfoLinear)
    {
        double dodt = -(lfoX - vp) / (kR53 * kC36 * (1.0 - kBeta));
        if (dodt > kSlew) dodt = kSlew;
        if (dodt < -kSlew) dodt = -kSlew;
        lfoO += dodt * dt;
        if (lfoO >= kOpHi) { lfoO = kOpHi; lfoLinear = false; }
        else if (lfoO <= kOpLo) { lfoO = kOpLo; lfoLinear = false; }
        lfoVm = kBeta * lfoO;
    }
    else
    {
        lfoVm += dt * (lfoX - lfoVm) / (kR53 * kC36);
        if ((lfoO >= kOpHi && lfoVm > vp) || (lfoO <= kOpLo && lfoVm < vp))
            lfoLinear = true;
    }

    // DEV wiper, loaded through R50 by the VCOin node (R51 to +15 V)
    const double w = (lfoY / ra + kSupply / rv) / (1.0 / ra + 1.0 / rb + 1.0 / rv);
    const double iR53 = (lfoX - lfoVm) / kR53;
    const double dx = ((lfoO - lfoX) / rm - (lfoX - lfoY) / kR54 - iR53) / kC39;
    const double dy = ((lfoX - lfoY) / kR54 - (lfoY - w) / ra) / kC37;
    lfoX += dx * dt;
    lfoY += dy * dt;
    vcoIn = w + (kSupply - w) * kR50 / rv;
}

//==============================================================================
double Adt::tickOs (double x)
{
    // U2: GAIN, its bandwidth and slew, clipping at its rails
    double v = u2Net.tick (c1Hp.tick (x * inG));
    v = u2Slew.tick (u2Gbw.tick (v));
    const double direct = swingClip (v, kOpHi, -kOpLo);

    // U4 ch.1 compressor: C12 dv/dt = -(i + G v), G from the rectifier
    const double i = c8Hp.tick (preEmph.tick (direct)) / kNe571R;
    double G = envC / (kVu * kNe571R);
    const double gMin = 1.0 / (kCompMaxGain * kNe571R);
    if (G < gMin) G = gMin;
    const double k2c = 2.0 * fsOs * kC12;
    vComp = (vComp * (k2c - G) - (i + iPrev)) / (k2c + G);
    vComp = swingClip (vComp, kCompHi, kCompLo);
    iPrev = i;
    envC += (std::fabs (vComp) - envC) * kEnvC;

    // Into the BBD
    v = sk2.tick (sk1.tick (c13Hp.tick (vComp)));
    const double bin = softClip (v, kBbdClip);
    {
        const double a = std::fabs (bin) / kBbdClip;
        if (a > peakBbdLevel) peakBbdLevel = a;
    }

    // MN3005: two-phase clock, one transfer per half period, 4096 stages back;
    // the two outputs (a half period apart) are averaged by TRIM2
    double step = (kStages / curDelay) / fsOs;          // transfers per oversampled sample
    if (step < 1.0e-3) step = 1.0e-3;
    const double inv = 1.0 / step;
    double pos = (1.0 - tickPhase) * inv;
    double last = 0.0, acc = 0.0;
    int guard = 0;
    while (pos <= 1.0 && guard < 64)
    {
        acc += bbdHeld * (pos - last);
        ring[ringPos] = (float) (lastBbdIn + (bin - lastBbdIn) * pos);
        bbdHeld = 0.5 * ((double) ring[(ringPos - kStages) & (kRing - 1)]
                       + (double) ring[(ringPos - kStages - 1) & (kRing - 1)]);
        ringPos = (ringPos + 1) & (kRing - 1);
        last = pos;
        pos += inv;
        ++guard;
    }
    acc += bbdHeld * (1.0 - last);
    tickPhase = 1.0 - (pos - 1.0) * step;
    if (tickPhase < 0.0) tickPhase = 0.0;
    if (tickPhase >= 1.0) tickPhase = 0.999999;
    lastBbdIn = bin;

    // Output network and the reconstruction filters
    v = c31Lp.tick (kBbdGain * acc * kR34 / (kR34 + kRtrimSrc));
    v = sk4.tick (sk3.tick (c34Hp.tick (v)));
    const double ein = c24Hp.tick (v);

    // U4 ch.2 expander: G cell driven by its own input's rectifier
    envE += (std::fabs (ein) - envE) * kEnvE;
    const double vExp = swingClip (-expFb.tick (ein * envE / kVu), kExpHi, kExpLo);
    const double dbl = c18Hp.tick (vExp);

    // Level indicator on the double
    ledLevel *= kLedDecay;
    if (dbl < kLedLo || dbl > kLedHi) ledLevel = 1.0;

    // MIX (the ADT switch takes the double out), C9, U3 follower, C5
    const double mixed = mixDry * direct + mixWet * adt * dbl;
    return c5Hp.tick (swingClip (c9Hp.tick (mixed), kOpHi, -kOpLo));
}

void Adt::process (const float* in, float* out, int numSamples)
{
    const double aK = 1.0 - smKnob, aW = 1.0 - smSwitch;
    const double dt = 1.0 / (fsBase * lfoSub);
    double xs[Oversampler4x::kFactor], ys[Oversampler4x::kFactor];

    for (int n = 0; n < numSamples; ++n)
    {
        gainP += (gainT - gainP) * aK;
        mixP += (mixT - mixP) * aK;
        modP += (modT - modP) * aK;
        devP += (devT - devP) * aK;
        delayP += (delayT - delayP) * aK;
        adt += (adtT - adt) * aW;
        inG += (inT - inG) * aK;
        outG += (outT - outG) * aK;

        // Pot-dependent networks: rebuilt every 16 samples while moving
        if (--countdown <= 0)
        {
            countdown = 16;
            if (std::fabs (gainP - gainApplied) > 1.0e-6) updateGain (gainP);
            if (std::fabs (mixP - mixApplied) > 1.0e-6) updateMix (mixP);
        }

        for (int k = 0; k < lfoSub; ++k) stepLfo (dt);
        curDelay = kStages / (2.0 * clockHz (delayP, vcoIn));

        float a = in[n];
        if (! (a == a)) a = 0.0f;
        up.upsample ((double) a, xs);
        for (int k = 0; k < Oversampler4x::kFactor; ++k)
            ys[k] = tickOs (xs[k]);
        out[n] = (float) (down.downsample (ys) * outG);
    }
}

} // namespace tbadt
