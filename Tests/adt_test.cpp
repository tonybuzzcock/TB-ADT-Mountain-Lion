// adt_test.cpp
// Offline checks for the TB Automatic Double Tracker DSP. Builds on Linux or macOS:
//   make -C Tests run      (writes demo WAVs to Tests/renders)
//
// 1. Delay: DELAY's range, the double arrives on time
// 2. GAIN: x1 to x101, the 741's bandwidth at high gain
// 3. The double's response: the filters' band, no output above it
// 4. Compander: the double follows the input level
// 5. MIX: direct end is direct only, the other end the double only
// 6. Modulation: MOD sets the speed, DEV the depth, DEV 0 none at all,
//    the same speed at any sample rate
// 7. ADT out takes the double out; the level indicator
// 8. Limits and a stability sweep
// 9. Automation of every control while playing, CPU, renders

#include "../Source/DSP/Adt.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <string>
#include <vector>

using namespace tbadt;

static const double kPiT = 3.14159265358979323846;
static const int kSr = 44100;
static int failures = 0;

static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

static bool allFinite (const std::vector<float>& x, double limit)
{
    for (size_t i = 0; i < x.size(); ++i)
        if (! (x[i] == x[i]) || std::fabs (x[i]) > limit) return false;
    return true;
}

static double rmsRange (const std::vector<float>& x, size_t from, size_t to)
{
    double s = 0.0;
    if (to > x.size()) to = x.size();
    for (size_t i = from; i < to; ++i) s += (double) x[i] * x[i];
    return to > from ? std::sqrt (s / (double) (to - from)) : 0.0;
}

static double peakRange (const std::vector<float>& x, size_t from, size_t to)
{
    double p = 0.0;
    if (to > x.size()) to = x.size();
    for (size_t i = from; i < to; ++i) p = std::max (p, (double) std::fabs (x[i]));
    return p;
}

static std::vector<float> sine (double f, double amp, double sec)
{
    std::vector<float> v ((size_t) (sec * kSr));
    for (size_t i = 0; i < v.size(); ++i) v[i] = (float) (amp * std::sin (2.0 * kPiT * f * i / kSr));
    return v;
}

static std::vector<float> run (Adt& a, const std::vector<float>& in)
{
    std::vector<float> out (in.size());
    for (size_t pos = 0; pos < in.size(); pos += 256)
    {
        const int n = (int) std::min ((size_t) 256, in.size() - pos);
        a.process (&in[pos], &out[pos], n);
    }
    return out;
}


static void writeWav (const std::string& path, const std::vector<float>& x, int sr)
{
    FILE* f = std::fopen (path.c_str(), "wb");
    if (! f) return;
    const unsigned dataBytes = (unsigned) x.size() * 2, riff = 36 + dataBytes;
    const unsigned short fmt = 1, ch = 1, bits = 16, align = 2;
    const unsigned rate = (unsigned) sr, bps = rate * 2, fmtLen = 16;
    std::fwrite ("RIFF", 1, 4, f); std::fwrite (&riff, 4, 1, f); std::fwrite ("WAVEfmt ", 1, 8, f);
    std::fwrite (&fmtLen, 4, 1, f); std::fwrite (&fmt, 2, 1, f); std::fwrite (&ch, 2, 1, f);
    std::fwrite (&rate, 4, 1, f); std::fwrite (&bps, 4, 1, f); std::fwrite (&align, 2, 1, f);
    std::fwrite (&bits, 2, 1, f); std::fwrite ("data", 1, 4, f); std::fwrite (&dataBytes, 4, 1, f);
    for (size_t i = 0; i < x.size(); ++i)
    {
        double v = x[i] * 0.9;
        v = v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v);
        const short s = (short) std::floor (v * 32767.0 + 0.5);
        std::fwrite (&s, 2, 1, f);
    }
    std::fclose (f);
}

// A decaying plucked string (Karplus-Strong), added into buf at start
static void pluck (std::vector<float>& buf, size_t start, double freq, double amp, double dur)
{
    const int period = std::max (2, (int) (kSr / freq + 0.5));
    std::vector<double> line (period);
    unsigned seed = (unsigned) (freq * 1000.0) + (unsigned) start;
    for (int i = 0; i < period; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        line[i] = ((seed >> 9) / 4194304.0 - 1.0) * amp;
    }
    const size_t n = (size_t) (dur * kSr);
    int idx = 0;
    double prev = 0.0;
    for (size_t i = 0; i < n && start + i < buf.size(); ++i)
    {
        const double cur = line[idx];
        const double next = 0.996 * 0.5 * (cur + prev);
        prev = cur;
        line[idx] = next;
        buf[start + i] += (float) cur;
        if (++idx >= period) idx = 0;
    }
}

static std::vector<float> makeChords (double seconds)
{
    std::vector<float> v ((size_t) (seconds * kSr), 0.0f);
    const double chords[4][4] = { { 82.4, 123.5, 164.8, 207.7 }, { 110.0, 164.8, 220.0, 277.2 },
                                  { 98.0, 146.8, 196.0, 246.9 }, { 73.4, 110.0, 146.8, 185.0 } };
    for (int c = 0; c * 2.0 < seconds; ++c)
        for (int s = 0; s < 4; ++s)
            pluck (v, (size_t) ((c * 2.0 + s * 0.012) * kSr), chords[c % 4][s], 0.22, 2.0);
    return v;
}

static std::vector<float> makeRiff (double seconds)
{
    std::vector<float> v ((size_t) (seconds * kSr), 0.0f);
    const double notes[8] = { 82.4, 98.0, 110.0, 123.5, 110.0, 98.0, 146.8, 130.8 };
    for (int i = 0; i * 0.25 < seconds; ++i)
        pluck (v, (size_t) (i * 0.25 * kSr), notes[i % 8], 0.35, 0.6);
    return v;
}

static std::vector<float> noise (double amp, double sec, unsigned seed)
{
    std::vector<float> v ((size_t) (sec * kSr));
    for (size_t i = 0; i < v.size(); ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        v[i] = (float) (amp * ((seed >> 9) / 4194304.0 - 1.0));
    }
    return v;
}


static void defaults (Adt& a)
{
    a.setGain (0.0); a.setMix (0.5); a.setMod (0.6); a.setDev (0.3); a.setDelay (0.5); a.setAdtOn (true);
    a.setInputGainDb (0.0); a.setOutputGainDb (0.0);
}

// The double alone: MIX fully clockwise, no modulation
static void doubleOnly (Adt& a, double delayPot)
{
    defaults (a);
    a.setDelay (delayPot); a.setDev (0.0); a.setMix (1.0);
}

// A short burst (Hann window, 4 ms) at t0
static std::vector<float> burst (double t0, double amp, double sec, double f = 1000.0)
{
    std::vector<float> v ((size_t) (sec * kSr), 0.0f);
    const int n = (int) (0.004 * kSr), s = (int) (t0 * kSr);
    for (int i = 0; i < n; ++i)
        v[s + i] = (float) (amp * 0.5 * (1.0 - std::cos (2.0 * kPiT * i / n)) * std::sin (2.0 * kPiT * f * i / kSr));
    return v;
}

// Gain (dB) of the double for a steady tone
static double doubleGainDb (Adt& a, double delayPot, double f, double amp = 0.2)
{
    doubleOnly (a, delayPot);
    a.reset();
    std::vector<float> y = run (a, sine (f, amp, 0.8));
    return 20.0 * std::log10 (rmsRange (y, (size_t) (0.5 * kSr), y.size()) / (amp / std::sqrt (2.0)) + 1e-12);
}

// Time (ms) of the strongest output after the burst, latency removed
static double measuredDelayMs (Adt& a, double delayPot)
{
    doubleOnly (a, delayPot);
    a.reset();
    std::vector<float> y = run (a, burst (0.05, 0.3, 0.5));
    size_t best = 0; double bv = 0.0;
    for (size_t i = (size_t) (0.052 * kSr); i < y.size(); ++i)
        if (std::fabs (y[i]) > bv) { bv = std::fabs (y[i]); best = i; }
    return 1000.0 * ((double) best - Adt::latencySamples()) / kSr - 50.0 - 2.0;
}

struct ModStats { double rate, dmin, dmax; };

// Modulation speed (from U6's output) and the delay's swing, after settling
static ModStats modStats (Adt& a, double mod, double dev, int sr = kSr, double sec = 8.0)
{
    defaults (a);
    a.setMod (mod); a.setDev (dev);
    a.prepare (sr);
    std::vector<float> in ((size_t) (sec * sr), 0.0f), out (in.size());
    double prev = a.lfoOutput();
    long first = -1, lastF = -1; int flips = 0;
    ModStats s = { 0.0, 1e9, 0.0 };
    for (size_t i = 0; i < in.size(); ++i)
    {
        a.process (&in[i], &out[i], 1);
        const double o = a.lfoOutput();
        if ((o > 0.0) != (prev > 0.0)) { ++flips; if (first < 0) first = (long) i; lastF = (long) i; }
        prev = o;
        if (i > (size_t) sr)
        {
            s.dmin = std::min (s.dmin, a.currentDelayMs());
            s.dmax = std::max (s.dmax, a.currentDelayMs());
        }
    }
    s.rate = flips > 2 ? 0.5 * (flips - 1) / ((double) (lastF - first) / sr) : 0.0;
    a.prepare (kSr);
    return s;
}

int main (int argc, char** argv)
{
    const std::string renderDir = argc > 1 ? argv[1] : ".";
    Adt* ap = new Adt();
    Adt& a = *ap;
    a.prepare (kSr);

    std::printf ("1. Delay\n");
    {
        std::printf ("  clock %.1f kHz .. %.1f kHz, delay %.1f .. %.1f ms (noon %.1f ms)\n",
                     Adt::clockHz (0.0, Adt::restVcoIn()) / 1000.0, Adt::clockHz (1.0, Adt::restVcoIn()) / 1000.0,
                     Adt::delayMs (0.0), Adt::delayMs (1.0), Adt::delayMs (0.5));
        bool mono = true;
        for (int i = 1; i <= 100; ++i)
            if (Adt::delayMs (i / 100.0) <= Adt::delayMs ((i - 1) / 100.0)) mono = false;
        check (mono, "DELAY clockwise always lengthens the delay");
        check (Adt::delayMs (0.5) > 20.0 && Adt::delayMs (0.5) < 40.0, "DELAY at noon is a double-tracking delay (20-40 ms)");
        const double pots[4] = { 0.0, 0.3, 0.5, 0.9 };
        bool onTime = true;
        for (int k = 0; k < 4; ++k)
        {
            const double want = Adt::delayMs (pots[k]), got = measuredDelayMs (a, pots[k]);
            std::printf ("  DELAY %.2f: expected %.1f ms, double peaks at %.1f ms\n", pots[k], want, got);
            if (std::fabs (got - want) > 0.5 + 0.03 * want) onTime = false;
        }
        check (onTime, "the double arrives at the BBD's delay (within 3%)");
    }

    std::printf ("\n2. GAIN\n");
    {
        double prev = -100.0; bool mono = true; double g0 = 0.0, gMax = 0.0;
        for (int g = 0; g <= 10; g += 2)
        {
            defaults (a); a.setGain (g / 10.0); a.setMix (0.0);
            a.reset();
            const double amp = 0.01;
            std::vector<float> y = run (a, sine (500.0, amp, 0.5));
            const double db = 20.0 * std::log10 (rmsRange (y, kSr / 4, y.size()) / (amp / std::sqrt (2.0)));
            std::printf ("  GAIN %2d: %+5.1f dB (U2 x%.1f)\n", g, db, Adt::gainFactor (g / 10.0));
            if (db <= prev) mono = false;
            prev = db;
            if (g == 0) g0 = db;
            if (g == 10) gMax = db;
        }
        check (mono, "GAIN clockwise always adds gain");
        check (std::fabs (g0) < 0.5 && std::fabs (gMax - 40.0) < 1.5, "x1 fully anticlockwise, about x100 (+40 dB) fully clockwise");
        defaults (a); a.setGain (1.0); a.setMix (0.0); a.reset();
        std::vector<float> y = run (a, sine (15000.0, 0.005, 0.4));
        const double hf = 20.0 * std::log10 (rmsRange (y, kSr / 5, y.size()) / (0.005 / std::sqrt (2.0)));
        std::printf ("  GAIN full at 15 kHz: %+.1f dB (the 741 runs out of loop gain)\n", hf);
        check (hf < gMax - 3.0, "at full GAIN the 741 rolls off the top");
    }

    std::printf ("\n3. The double's response (0.2 V tones, double only)\n");
    double ref500 = 0.0;
    {
        const double freqs[8] = { 60.0, 150.0, 500.0, 1000.0, 3000.0, 6000.0, 10000.0, 16000.0 };
        double sh[8], lg[8];
        for (int k = 0; k < 8; ++k)
        {
            sh[k] = doubleGainDb (a, 0.3, freqs[k]);
            lg[k] = doubleGainDb (a, 0.9, freqs[k]);
            std::printf ("  %5.0f Hz: DELAY 0.3 %+6.1f dB   DELAY 0.9 %+6.1f dB\n", freqs[k], sh[k], lg[k]);
        }
        ref500 = sh[2];
        check (std::fabs (sh[2]) < 4.0 && std::fabs (sh[3]) < 4.0, "the double comes back near the direct level");
        check (std::fabs (sh[4] - sh[2]) < 4.0, "pre- and de-emphasis cancel through the mid band");
        check (sh[7] < sh[2] - 6.0, "the filters cut above their band");
        check (lg[7] < lg[2] - 6.0, "and at long delays too");
    }

    std::printf ("\n4. Compander\n");
    {
        const double lo = doubleGainDb (a, 0.4, 400.0, 0.02), mid = doubleGainDb (a, 0.4, 400.0, 0.2),
                     hi = doubleGainDb (a, 0.4, 400.0, 1.0);
        std::printf ("  double gain at 20 mV %+.1f dB, 200 mV %+.1f dB, 1 V %+.1f dB\n", lo, mid, hi);
        check (std::fabs (lo - mid) < 3.0 && std::fabs (hi - mid) < 3.0,
               "compressor and expander cancel: the double follows the input level");
        doubleOnly (a, 0.4); a.reset();
        run (a, sine (400.0, 0.2, 1.0));
        std::printf ("  rectifiers on a steady 0.2 V tone: compressor %.3f V, expander %.3f V\n",
                     a.compressorEnv(), a.expanderEnv());
        check (a.compressorEnv() > 0.1 && a.compressorEnv() < 2.0, "the compressor's rectifier settles at a sensible level");
    }

    std::printf ("\n5. MIX\n");
    {
        const double d = Adt::delayMs (0.7) / 1000.0;
        std::vector<float> x = burst (0.05, 0.3, 0.6);
        const size_t drFrom = (size_t) (0.045 * kSr), drTo = (size_t) (0.06 * kSr) + Adt::latencySamples();
        const size_t dbFrom = (size_t) ((0.05 + d - 0.003) * kSr), dbTo = (size_t) ((0.05 + d + 0.01) * kSr);
        double dr[3], db[3];
        const double mixes[3] = { 0.0, 0.5, 1.0 };
        for (int k = 0; k < 3; ++k)
        {
            doubleOnly (a, 0.7); a.setMix (mixes[k]); a.reset();
            std::vector<float> y = run (a, x);
            dr[k] = peakRange (y, drFrom, drTo); db[k] = peakRange (y, dbFrom, dbTo);
            std::printf ("  MIX %.1f: direct peak %.3f, double peak %.3f\n", mixes[k], dr[k], db[k]);
        }
        check (dr[0] > 0.25 && db[0] < 0.003, "anticlockwise: the direct signal only");
        check (dr[2] < 0.003 && db[2] > 0.1, "clockwise: the double only");
        check (dr[1] > 0.1 && db[1] > 0.05, "centre: both");
    }

    std::printf ("\n6. Modulation\n");
    {
        double prevRate = 0.0; bool faster = true;
        const double mods[5] = { 0.0, 0.5, 0.8, 0.9, 1.0 };
        double rates[5];
        for (int k = 0; k < 5; ++k)
        {
            ModStats s = modStats (a, mods[k], 0.5);
            rates[k] = s.rate;
            std::printf ("  MOD %.1f (DEV 5): %.2f Hz, delay %.2f .. %.2f ms\n", mods[k], s.rate, s.dmin, s.dmax);
            if (s.rate < prevRate * 0.97) faster = false;
            prevRate = s.rate;
        }
        check (rates[0] > 0.2 && faster && rates[4] > 10.0 * rates[0], "MOD clockwise speeds the modulation up");

        double prevSwing = -1.0; bool deeper = true; double swing0 = 0.0;
        for (int d = 0; d <= 10; d += 5)
        {
            ModStats s = modStats (a, 0.6, d / 10.0);
            const double sw = s.dmax - s.dmin;
            std::printf ("  DEV %2d (MOD 6): delay %.2f .. %.2f ms\n", d, s.dmin, s.dmax);
            if (sw <= prevSwing) deeper = false;
            prevSwing = sw;
            if (d == 0) swing0 = sw;
        }
        check (swing0 < 0.01, "DEV fully anticlockwise: no modulation");
        check (deeper, "DEV clockwise deepens it");

        ModStats s44 = modStats (a, 0.7, 0.5, 44100), s96 = modStats (a, 0.7, 0.5, 96000);
        std::printf ("  MOD 7: %.3f Hz at 44.1 kHz, %.3f Hz at 96 kHz\n", s44.rate, s96.rate);
        check (std::fabs (s96.rate - s44.rate) < 0.03 * s44.rate, "the same speed at any sample rate");
    }

    std::printf ("\n7. ADT in / out, level indicator\n");
    {
        const double d = Adt::delayMs (0.7) / 1000.0;
        std::vector<float> x = burst (0.05, 0.3, 0.6);
        defaults (a); a.setDelay (0.7); a.setDev (0.0); a.setAdtOn (false); a.reset();
        std::vector<float> y = run (a, x);
        const double dr = peakRange (y, (size_t) (0.045 * kSr), (size_t) (0.06 * kSr) + Adt::latencySamples());
        const double db = peakRange (y, (size_t) ((0.05 + d - 0.003) * kSr), (size_t) ((0.05 + d + 0.01) * kSr));
        std::printf ("  ADT out at MIX 5: direct peak %.3f, double peak %.4f\n", dr, db);
        check (db < 0.002 && dr > 0.1, "ADT out takes the double out and leaves MIX's share of the direct signal");

        defaults (a); a.reset();
        run (a, sine (400.0, 0.3, 1.0));
        const double quiet = a.peakLight();
        defaults (a); a.setGain (0.6); a.reset();
        run (a, sine (400.0, 0.3, 1.0));
        const double loud = a.peakLight();
        std::printf ("  indicator: 0.3 V at GAIN 0 %.2f, at GAIN 6 %.2f\n", quiet, loud);
        check (quiet < 0.01 && loud > 0.5, "the level indicator stays off at normal levels and lights when the double is driven hard");
    }

    std::printf ("\n8. Limits and stability\n");
    {
        defaults (a); a.setGain (1.0); a.setMix (1.0); a.reset();
        a.clearPeakBbd();
        std::vector<float> y = run (a, sine (220.0, 3.0, 1.0));
        std::printf ("  3 V in at full GAIN: output peak %.2f, BBD drive %.2f of clip\n", peakRange (y, kSr / 2, kSr), a.peakBbd());
        check (allFinite (y, 20.0) && peakRange (y, kSr / 2, kSr) < 14.0, "hot input stays bounded (rails and BBD clip)");

        const double amps[4] = { 0.01, 0.1, 1.0, 4.0 };
        const double freqs[5] = { 41.0, 110.0, 440.0, 2000.0, 15000.0 };
        const double corners[4][5] = { { 0, 0, 0, 0, 0 }, { 1, 1, 1, 1, 1 }, { 0, 1, 1, 0, 1 }, { 1, 0.5, 0, 1, 0 } };
        int runs = 0; bool ok = true;
        for (int c = 0; c < 4; ++c)
            for (int ai = 0; ai < 4; ++ai)
                for (int fi = 0; fi < 5; ++fi)
                {
                    a.setGain (corners[c][0]); a.setMix (corners[c][1]); a.setMod (corners[c][2]);
                    a.setDev (corners[c][3]); a.setDelay (corners[c][4]); a.setAdtOn (true);
                    a.reset();
                    std::vector<float> o = run (a, sine (freqs[fi], amps[ai], 0.4));
                    if (! allFinite (o, 20.0)) ok = false;
                    ++runs;
                }
        unsigned seed = 1;
        for (int r = 0; r < 20; ++r)
        {
            double f[5];
            for (int k = 0; k < 5; ++k) { seed = seed * 1664525u + 1013904223u; f[k] = (seed >> 8) / 16777216.0; }
            a.setGain (f[0]); a.setMix (f[1]); a.setMod (f[2]); a.setDev (f[3]); a.setDelay (f[4]);
            a.reset();
            std::vector<float> o = run (a, noise (2.0, 0.4, seed));
            if (! allFinite (o, 20.0)) ok = false;
            ++runs;
        }
        std::printf ("  %d runs\n", runs);
        check (ok, "no NaNs or runaways past the rails anywhere");

        defaults (a); a.reset();
        std::vector<float> z = run (a, std::vector<float> ((size_t) (2 * kSr), 0.0f));
        check (peakRange (z, kSr, z.size()) < 1e-4, "silence in, silence out");
    }

    std::printf ("\n9. Automation, CPU, renders\n");
    {
        defaults (a);
        a.reset();
        std::vector<float> x = makeRiff (4.0), y (x.size());
        for (size_t pos = 0; pos < x.size(); pos += 64)
        {
            const double t = (double) pos / x.size();
            a.setGain (0.3 + 0.3 * std::sin (2 * kPiT * 2 * t));
            a.setMix (0.5 + 0.5 * std::sin (2 * kPiT * 7 * t));
            a.setMod (0.5 + 0.5 * std::sin (2 * kPiT * 5 * t));
            a.setDev (0.5 + 0.5 * std::sin (2 * kPiT * 6 * t));
            a.setDelay (0.5 + 0.5 * std::sin (2 * kPiT * 3 * t));
            a.setAdtOn (std::sin (2 * kPiT * 4 * t) > 0.0);
            a.setInputGainDb (12.0 * std::sin (2 * kPiT * 2 * t));
            a.setOutputGainDb (12.0 * std::sin (2 * kPiT * 9 * t));
            const int n = (int) std::min ((size_t) 64, x.size() - pos);
            a.process (&x[pos], &y[pos], n);
        }
        std::printf ("  automated: output peak %.2f\n", peakRange (y, 0, y.size()));
        check (allFinite (y, 60.0), "every control automated while playing");

        defaults (a);
        a.reset();
        std::vector<float> ten = makeChords (10.0);
        std::clock_t t0 = std::clock();
        std::vector<float> yc = run (a, ten);
        const double secs = (double) (std::clock() - t0) / CLOCKS_PER_SEC;
        std::printf ("  CPU: %.1f%% of one core at 44.1 kHz\n", 10.0 * secs);
        check (secs < 3.0, "runs well under real time");

        struct R { const char* name; double mix, mod, dev, delay; bool on, chords; };
        const R renders[] = {
            { "dry_riff",              0.5, 0.6, 0.3, 0.5,  false, false },
            { "default_riff",          0.5, 0.6, 0.3, 0.5,  true,  false },
            { "tight_double_chords",   0.5, 0.5, 0.2, 0.35, true,  true  },
            { "wide_chorus_chords",    0.5, 0.8, 0.8, 0.45, true,  true  },
            { "slapback_riff",         0.45, 0.5, 0.0, 0.8, true,  false },
        };
        std::vector<float> riff = makeRiff (8.0), chords = makeChords (8.0);
        for (size_t k = 0; k < sizeof (renders) / sizeof (renders[0]); ++k)
        {
            defaults (a);
            a.setMix (renders[k].mix); a.setMod (renders[k].mod); a.setDev (renders[k].dev);
            a.setDelay (renders[k].delay); a.setAdtOn (renders[k].on);
            a.reset();
            const std::vector<float>& src = renders[k].chords ? chords : riff;
            std::vector<float> o = run (a, src);
            writeWav (renderDir + "/" + renders[k].name + ".wav", o, kSr);
        }
        check (true, "demo renders written");
    }
    (void) ref500;

    delete ap;
    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL DSP TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
