
#include <AudioEffectCompWDRC_F32.h>  //for CHA_DSL and CHA_WDRC data types

//from GHA_Demo.c
// DSL prescription - (first subject, left ear) from LD_RX.mat
static BTNRH_WDRC::CHA_DSL dsl = {20,  //attack (ms)
  510,  //release (ms)
  115, //dB SPL for input signal at 0 dBFS.  Needs to be tailored to mic, spkrs, and mic gain.
  0, // 0=left, 1=right
  8, //num channels...ignored.  10 is always assumed
  {250.0, 500.0, 750.0, 1000.0, 1500.0, 2000.0, 3000.0},   // cross frequencies (Hz)...FOR IIR FILTERING, THESE VALUES ARE IGNORED!!!
  {0.57, 0.57, 0.57, 0.57, 0.57, 0.57, 0.57, 0.57},   // compression ratio for low-SPL region (ie, the expander..values should be < 1.0)
  {45.0, 45.0, 33.0, 32.0, 36.0, 34.0, 36.0, 40.0},   // expansion-end kneepoint
  {0.f, 0.f, 5.f, 6.f, 8.f, 8.f, 11.f, 10.f},   // compression-start gain
  {1.1f, 1.1f, 1.2f, 1.5f, 1.9f, 2.3f, 2.3f, 2.8f},   // compression ratio
  {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0},   // compression-start kneepoint (input dB SPL)
  {90.f, 90.f, 90.f, 90.f, 90.f, 91.f, 92.f, 93.f}    // output limiting threshold (comp ratio 10)
};

static BTNRH_WDRC::CHA_DSL dslr = {20,  //attack (ms)
  510,  //release (ms)
  115, //dB SPL for input signal at 0 dBFS.  Needs to be tailored to mic, spkrs, and mic gain.
  1, // 0=left, 1=right
  8, //num channels...ignored.  10 is always assumed
  {250.0, 500.0, 750.0, 1000.0, 1500.0, 2000.0, 3000.0},   // cross frequencies (Hz)...FOR IIR FILTERING, THESE VALUES ARE IGNORED!!!
  {0.57, 0.57, 0.57, 0.57, 0.57, 0.57, 0.57, 0.57},   // compression ratio for low-SPL region (ie, the expander..values should be < 1.0)
  {45.0, 45.0, 33.0, 32.0, 36.0, 34.0, 36.0, 40.0},   // expansion-end kneepoint
  {0.f, 0.f, 5.f, 6.f, 8.f, 8.f, 11.f, 10.f},   // compression-start gain
  {1.1f, 1.1f, 1.2f, 1.5f, 1.9f, 2.3f, 2.3f, 2.8f},   // compression ratio
  {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0},   // compression-start kneepoint (input dB SPL)
  {90.f, 90.f, 90.f, 90.f, 90.f, 91.f, 92.f, 93.f}    // output limiting threshold (comp ratio 10)
};

// version = "FLAT-M/SLOW"
