#include <DaisyDuino.h>
#include <math.h>

using namespace daisysp;
using namespace daisy;

// ---------- User options ----------
constexpr float BASE_FREQ_HZ = 65.406f; // C2 (change to taste: C1=32.703f, C3=130.813f)

// ---------- Pin map ----------
constexpr uint8_t PIN_POT1 = A2; // CW=max (we invert in code)
constexpr uint8_t PIN_POT2 = A3;
constexpr uint8_t PIN_POT3 = A4;
constexpr uint8_t PIN_POT4 = A5;

constexpr uint8_t PIN_CV1  = A0; // CV1 (±5V scaled to 0..3.3V), used for pitch in OSC mode only
constexpr uint8_t PIN_CV2  = A1; // Used for tap in Ping-Pong only

constexpr uint8_t PIN_BTN1 = D3; // mode MSB
constexpr uint8_t PIN_BTN2 = D1; // mode LSB

// ---------- Hardware ----------
DaisyHardware patch;
float samplerate = 48000.f;

// ---------- DSP ----------
DSY_SDRAM_BSS static ReverbSc verb;
DSY_SDRAM_BSS static DelayLine<float, 48000> delay_l, delay_r; // for ping-pong
DSY_SDRAM_BSS static DelayLine<float, 48000> tapeL, tapeR;     // for tape echo

static Oscillator osc_l, osc_r;
static Overdrive  drive_l, drive_r;

// --- simple one-pole LPF for tape feedback tone ---
struct OnePoleLP {
  float y = 0.f, a = 0.1f;
  inline float proc(float x) { y += a * (x - y); return y; }
  inline void   setCutoff(float sr, float hz)
  {
    hz = fclamp(hz, 20.f, sr * 0.45f);
    float x = expf(-2.f * 3.14159265f * hz / sr);
    a = 1.f - x;
  }
} toneL, toneR;

// ---------- Smoothed controls ----------
float pot1 = 0.f, pot2 = 0.f, pot3 = 0.f, pot4 = 0.f; // 0..1 (after pot inversion)
float cv1_volts = 0.f, cv2_volts = 0.f;               // true jack volts (−5..+5)

Switch button1, button2;

// ---------- Tap / clock via CV2 (for ping-pong) ----------
uint32_t last_tap_ms     = 0;
float    tap_delay_samps = 24000.0f; // default ~500 ms @48k
bool     cv2_gate_state  = false;
constexpr float TAP_HIGH_V = 2.0f; // threshold in Vin volts
constexpr float TAP_LOW_V  = 1.0f;

// ---------- Tape Echo LFOs (wow/flutter) ----------
float wowPhaseL = 0.f, wowPhaseR = 0.33f;
float fltPhaseL = 0.5f, fltPhaseR = 0.83f;
float wowInc = 0.f, fltInc = 0.f;

// ---------- State ----------
enum EffectState { REVERB_MODE, PING_PONG_MODE, TAPE_ECHO_MODE, OSCILLATOR_MODE };
EffectState current_effect = REVERB_MODE;

// ---------- Calibration (from your DMM data) ----------
// ADC Vout = A + B * Vin, where Vin is jack volts (−5..+5)
constexpr float CAL_A_V   = 1.68f;   // intercept at Vin=0 V
constexpr float CAL_B_VPV = -0.33f;  // slope (Vout volts per 1 V of Vin)

// ---------- Helpers ----------
inline float ReadADC16(uint8_t pin) { return analogRead(pin) / 65535.0f; } // 0..1
inline float ReadPot(uint8_t pin)   { return 1.0f - ReadADC16(pin); }      // invert so CW=max

// Convert ADC 0..1 to jack Vin (−5..+5 V)
inline float AdcNormToVin(float a_norm)
{
  const float vout = 3.3f * a_norm;           // volts at MCU pin
  return (vout - CAL_A_V) / CAL_B_VPV;        // back-solve for Vin
}

inline void DoTap(uint32_t now_ms)
{
  const uint32_t dt = now_ms - last_tap_ms; // ms
  if (dt > 50 && dt < 2000)                 // 30–1200 BPM
    tap_delay_samps = (dt / 1000.0f) * samplerate;
  last_tap_ms = now_ms;
}

// Discrete octave selector (−2..+2) from Pot1
inline int PotToOctave(float p)
{
  if (p < 0.2f) return -2;
  else if (p < 0.4f) return -1;
  else if (p < 0.6f) return 0;
  else if (p < 0.8f) return +1;
  else return +2;
}

// ---------- Audio ----------
void AudioCallback(float **in, float **out, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    float dryL = in[0][i];
    float dryR = in[1][i];
    float wetL = 0.f, wetR = 0.f;

    switch (current_effect)
    {
      case REVERB_MODE:
      {
        // Pot1 = mix, Pot2 = decay, Pot3 = send, Pot4 = LP tone (set in loop)
        float sendL = dryL * pot3;
        float sendR = dryR * pot3;
        verb.Process(sendL, sendR, &wetL, &wetR);
        float m = pot1;
        out[0][i] = (1.f - m) * dryL + m * wetL;
        out[1][i] = (1.f - m) * dryR + m * wetR;
      }
      break;

      case PING_PONG_MODE:
      {
        static float smoothed_time = 0.f;
        fonepole(smoothed_time, tap_delay_samps, 0.0015f);

        float d = fclamp(smoothed_time, 10.f, 47990.f);
        delay_l.SetDelay(d);
        delay_r.SetDelay(d);

        float dl = delay_l.Read();
        float dr = delay_r.Read();

        float spread = pot4; // 0=mono feedback, 1=crossfeed ping-pong
        float fb_l = dl * (1.f - spread) + dr * spread;
        float fb_r = dr * (1.f - spread) + dl * spread;

        float feedback = pot3; // feedback from pot3
        delay_l.Write(dryL + fb_l * feedback);
        delay_r.Write(dryR + fb_r * feedback);

        float m = pot1;
        out[0][i] = (1.f - m) * dryL + m * dl;
        out[1][i] = (1.f - m) * dryR + m * dr;
      }
      break;

      case TAPE_ECHO_MODE:
      {
        // Time and tone are set in loop. Add wow/flutter by modulating SetDelay slightly.
        // Read delayed samples
        float tl = tapeL.Read();
        float tr = tapeR.Read();

        // Filter feedback for tone
        float fb_l = toneL.proc(tl) * pot3;
        float fb_r = toneR.proc(tr) * pot3;

        // Write with feedback
        tapeL.Write(dryL + fb_l);
        tapeR.Write(dryR + fb_r);

        // Wet = delayed
        wetL = tl;
        wetR = tr;

        float m = pot1;
        out[0][i] = (1.f - m) * dryL + m * wetL;
        out[1][i] = (1.f - m) * dryR + m * wetR;

        // Advance LFOs and modulate delay times (small range, safe clamp)
        wowPhaseL += wowInc; if (wowPhaseL >= 1.f) wowPhaseL -= 1.f;
        wowPhaseR += wowInc; if (wowPhaseR >= 1.f) wowPhaseR -= 1.f;
        fltPhaseL += fltInc; if (fltPhaseL >= 1.f) fltPhaseL -= 1.f;
        fltPhaseR += fltInc; if (fltPhaseR >= 1.f) fltPhaseR -= 1.f;

        // (loop-end modulation handled in loop() by updating base time; here we keep things simple)
      }
      break;

      case OSCILLATOR_MODE:
      {
        // Saw oscillator, lower base freq, discrete octave + fine tune
        static int last_oct = 0;
        int   oct_sel  = PotToOctave(pot1);
        if (oct_sel != last_oct) last_oct = oct_sel;

        // CV1 1V/Oct with deadband and clamp
        float octs_cv = (fabsf(cv1_volts) < 0.02f) ? 0.f : cv1_volts; // ~±20 mV deadband
        octs_cv = fclamp(octs_cv, -3.f, +3.f);

        // Fine tune ±50 cents
        float cents = fmap(pot2, -50.f, 50.f);
        float cents_factor = powf(2.f, cents / 1200.f);

        float base_oct = (float)oct_sel + octs_cv;
        float freq_base = BASE_FREQ_HZ * powf(2.f, base_oct);

        float detune_hz = fmap(pot3, 0.f, 5.f);

        float freqL = fclamp(freq_base * cents_factor, 20.f, 20000.f);
        float freqR = fclamp((freq_base + detune_hz) * cents_factor, 20.f, 20000.f);

        osc_l.SetFreq(freqL);
        osc_r.SetFreq(freqR);

        drive_l.SetDrive(pot4);
        drive_r.SetDrive(pot4);

        float l = drive_l.Process(osc_l.Process());
        float r = drive_r.Process(osc_r.Process());
        out[0][i] = l;
        out[1][i] = r;
      }
      break;
    }
  }
}

// ---------- Setup / Loop ----------
void setup()
{
  patch = DAISY.init(DAISY_PATCH, AUDIO_SR_48K);
  samplerate = DAISY.get_samplerate();
  analogReadResolution(16);

  // Reverb
  verb.Init(samplerate);

  // Delays
  delay_l.Init();
  delay_r.Init();
  tapeL.Init();
  tapeR.Init();

  // Tape tone defaults
  toneL.setCutoff(samplerate, 6000.f);
  toneR.setCutoff(samplerate, 6000.f);

  // Wow/flutter default rates
  wowInc = 0.3f / samplerate; // ~0.3 Hz
  fltInc = 4.5f / samplerate; // ~4.5 Hz

  // Oscillator
  osc_l.Init(samplerate);
  osc_r.Init(samplerate);
  osc_l.SetWaveform(Oscillator::WAVE_SAW);
  osc_r.SetWaveform(Oscillator::WAVE_SAW);
  osc_l.SetAmp(0.5f);
  osc_r.SetAmp(0.5f);

  drive_l.Init();
  drive_r.Init();

  // UI
  button1.Init(samplerate, true, PIN_BTN1, INPUT_PULLUP);
  button2.Init(samplerate, true, PIN_BTN2, INPUT_PULLUP);

  DAISY.begin(AudioCallback);
}

void loop()
{
  patch.ProcessAllControls();
  button1.Debounce();
  button2.Debounce();

  // Mode index (00=REV, 01=PING, 10=TAPE, 11=OSC)
  EffectState prev = current_effect;
  int idx = (button1.Pressed() ? 2 : 0) | (button2.Pressed() ? 1 : 0);
  current_effect = static_cast<EffectState>(idx);

  if (prev != current_effect)
  {
    switch (current_effect)
    {
      case REVERB_MODE:
        verb.Init(samplerate);
        break;
      case PING_PONG_MODE:
        delay_l.Reset();
        delay_r.Reset();
        break;
      case TAPE_ECHO_MODE:
        tapeL.Reset();
        tapeR.Reset();
        break;
      case OSCILLATOR_MODE:
        osc_l.Reset();
        osc_r.Reset();
        break;
    }
  }

  // Pots (invert so CW=max)
  fonepole(pot1, ReadPot(PIN_POT1), 0.005f);
  fonepole(pot2, ReadPot(PIN_POT2), 0.005f);
  fonepole(pot3, ReadPot(PIN_POT3), 0.005f);
  fonepole(pot4, ReadPot(PIN_POT4), 0.005f);

  // CVs to true volts (−5..+5) using calibration
  float cv1_v_raw = AdcNormToVin(ReadADC16(PIN_CV1));
  float cv2_v_raw = AdcNormToVin(ReadADC16(PIN_CV2));
  fonepole(cv1_volts, cv1_v_raw, 0.005f);
  fonepole(cv2_volts, cv2_v_raw, 0.005f);

  // Per-mode non-audio-rate params
  switch (current_effect)
  {
    case REVERB_MODE:
    {
      verb.SetFeedback(fmap(pot2, 0.70f, 0.98f));            // decay
      verb.SetLpFreq(fmap(pot4, 1000.f, 18000.f));           // tone
      // Pot3 used as send in audio callback
    }
    break;

    case PING_PONG_MODE:
    {
      // Tap from CV2 volts (edge detect with hysteresis)
      uint32_t now = daisy::System::GetNow();
      if (!cv2_gate_state && cv2_volts >= TAP_HIGH_V) { cv2_gate_state = true; DoTap(now); }
      else if (cv2_gate_state && cv2_volts <= TAP_LOW_V) { cv2_gate_state = false; }

      // Fallback to manual time (Pot2) if no recent taps
      bool tapped_recently = (now - last_tap_ms) < 2000; // 2 s
      if (!tapped_recently)
        tap_delay_samps = fmap(pot2, 10.f, 47990.f);
      else
        tap_delay_samps = fclamp(tap_delay_samps, 10.f, 47990.f);
    }
    break;

    case TAPE_ECHO_MODE:
    {
      // Time (samples): 50 ms .. 1000 ms (safe inside 48k buffer)
      float minS = samplerate * 0.050f; // ~2400
      float maxS = samplerate * 1.000f; // 48000
      float baseS = fclamp(fmap(pot2, minS, maxS), 10.f, 47990.f);

      // Tone 800..12k
      float toneHz = fmap(pot4, 800.f, 12000.f);
      toneL.setCutoff(samplerate, toneHz);
      toneR.setCutoff(samplerate, toneHz);

      // Apply wow/flutter modulation (few ms) to each channel
      // Wow: slow ±3 ms, Flutter: fast ±0.6 ms
      float wowDepthS = samplerate * 0.003f;
      float fltDepthS = samplerate * 0.0006f;

      float modL = baseS
                 + wowDepthS * sinf(2.f * 3.14159265f * wowPhaseL)
                 + fltDepthS * sinf(2.f * 3.14159265f * fltPhaseL);

      float modR = baseS
                 + wowDepthS * sinf(2.f * 3.14159265f * wowPhaseR)
                 + fltDepthS * sinf(2.f * 3.14159265f * fltPhaseR);

      modL = fclamp(modL, 10.f, 47990.f);
      modR = fclamp(modR, 10.f, 47990.f);

      tapeL.SetDelay(modL);
      tapeR.SetDelay(modR);

      // Advance LFO phases
      wowPhaseL += wowInc; if (wowPhaseL >= 1.f) wowPhaseL -= 1.f;
      wowPhaseR += wowInc; if (wowPhaseR >= 1.f) wowPhaseR -= 1.f;
      fltPhaseL += fltInc; if (fltPhaseL >= 1.f) fltPhaseL -= 1.f;
      fltPhaseR += fltInc; if (fltPhaseR >= 1.f) fltPhaseR -= 1.f;
    }
    break;

    case OSCILLATOR_MODE:
      // handled in audio callback
    break;
  }
}


