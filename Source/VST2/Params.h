// Params.h
// Parameter table shared by the VST wrapper, the editor and the preset store.
//
// Front-panel controls first, in the unit's order (GAIN, MIX, MOD, DEV,
// DELAY, ADT in / out), then the plugin-only controls in the editor's
// bottom strip.

#ifndef TBADT_PARAMS_H
#define TBADT_PARAMS_H

namespace tbadt
{

enum Param
{
    // Front panel
    kGain = 0, kMix, kMod, kDev, kDelay, kAdt,
    // Plugin only (bottom strip of the editor)
    kInput, kOutput,
    kNumParams
};

enum Kind { kKnob, kGainDb, kDelayMs, kSwitch, kDb12, kDb24 };

const char* const kParamNames[kNumParams] = {
    "Gain", "Mix", "Mod", "Dev", "Delay", "ADT",
    "Input", "Output"
};

const int kParamKind[kNumParams] = {
    kGainDb, kKnob, kKnob, kKnob, kDelayMs, kSwitch,
    kDb12, kDb24
};

const float kDefaults[kNumParams] = {
    0.0f, 0.5f, 0.6f, 0.3f, 0.5f, 1.0f,
    0.5f, 0.5f
};

const double kInputRangeDb  = 12.0;
const double kOutputRangeDb = 24.0;

inline bool isOn (float v) { return v >= 0.5f; }

} // namespace tbadt

#endif
