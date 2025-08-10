#include <DaisyDuino.h>

using namespace daisysp;
using namespace daisy;

// Hardware object
DaisyHardware patch;
float samplerate;

// --- Effect Objects and Global Variables ---
static ReverbSc verb;
DSY_SDRAM_BSS static DelayLine<float, 48000> delay_l, delay_r;
static Chorus chorus_l, chorus_r;
static GateIn clock_input;
// REMOVED: static Gate gate_l, gate_r;
Switch button1, button2;

// --- Manual Tap Tempo Variables ---
uint32_t last_tap_millis = 0;
float    tap_delay_time_samples = 0;

// --- Control Variables ---
float reverb_dry_level, reverb_send, reverb_feedback, reverb_lp_freq;
float delay_mix, delay_time, delay_feedback, delay_send;
float dr_mix, dr_delay_time, dr_delay_feedback, dr_reverb_decay;
float chorus_mix, chorus_rate, chorus_depth, chorus_feedback;
// --- Smoothed Variables ---
float smoothed_delay_time, smoothed_dr_delay_time;
float smoothed_reverb_cv;

// --- State Management ---
enum EffectState
{
  REVERB_MODE,
  DELAY_MODE,
  DELAY_REVERB_MODE,
  CHORUS_MODE
};
EffectState current_effect;


// --- Helper Functions ---
float CtrlVal(uint8_t pin) {
  analogReadResolution(16);
  return (1.0f - (analogRead(pin) / 65535.f));
}
float ScaleCvLFO(float raw_adc) {
    float clamped_adc = fclamp(raw_adc, 0.25f, 0.75f);
    return (clamped_adc - 0.75f) / (0.25f - 0.75f);
}
float ScaleCvClock(float raw_adc) {
    float clamped_adc = fclamp(raw_adc, 0.25f, 0.5f);
    return (clamped_adc - 0.5f) / (0.25f - 0.5f);
}
void ProcessTap() {
  uint32_t now = daisy::System::GetNow();
  uint32_t interval = now - last_tap_millis;
  if (interval > 50 && interval < 2000) {
    tap_delay_time_samples = (interval / 1000.0f) * samplerate;
  }
  last_tap_millis = now;
}

// --- Audio Callback ---
void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    float dryL = in[0][i];
    float dryR = in[1][i];

    // --- Manual Noise Gate Logic ---
    const float GATE_THRESHOLD = 0.0015f; // Threshold is approx -56dB
    if (fabsf(dryL) < GATE_THRESHOLD) {
        dryL = 0.0f;
    }
    if (fabsf(dryR) < GATE_THRESHOLD) {
        dryR = 0.0f;
    }

    float wetL = 0.f;
    float wetR = 0.f;

    switch (current_effect) {
      case REVERB_MODE: {
        float sendL = dryL * reverb_send;
        float sendR = dryR * reverb_send;
        verb.Process(sendL, sendR, &wetL, &wetR);
        out[0][i] = (dryL * reverb_dry_level) + wetL;
        out[1][i] = (dryR * reverb_dry_level) + wetR;
        break;
      }
      case DELAY_MODE: {
        fonepole(smoothed_delay_time, delay_time, .0002f);
        delay_l.SetDelay(smoothed_delay_time);
        delay_r.SetDelay(smoothed_delay_time);
        float sendL = dryL * delay_send;
        float sendR = dryR * delay_send;
        float delayed_out_L = delay_l.Read();
        float delayed_out_R = delay_r.Read();
        float feedback_signal_L = sendL + (delayed_out_L * delay_feedback);
        float feedback_signal_R = sendR + (delayed_out_R * delay_feedback);
        fonepole(feedback_signal_L, fclamp(feedback_signal_L, -1.0f, 1.0f), .001f);
        fonepole(feedback_signal_R, fclamp(feedback_signal_R, -1.0f, 1.0f), .001f);
        delay_l.Write(feedback_signal_L);
        delay_r.Write(feedback_signal_R);
        wetL = delayed_out_L;
        wetR = delayed_out_R;
        out[0][i] = (dryL * (1.0f - delay_mix)) + (wetL * delay_mix);
        out[1][i] = (dryR * (1.0f - delay_mix)) + (wetR * delay_mix);
        break;
      }
      case DELAY_REVERB_MODE: {
        fonepole(smoothed_dr_delay_time, dr_delay_time, .0002f);
        delay_l.SetDelay(smoothed_dr_delay_time);
        delay_r.SetDelay(smoothed_dr_delay_time);
        float delay_out_L = delay_l.Read();
        float delay_out_R = delay_r.Read();
        float feedback_signal_L = (dryL * 0.7f) + (delay_out_L * dr_delay_feedback);
        float feedback_signal_R = (dryR * 0.7f) + (delay_out_R * dr_delay_feedback);
        fonepole(feedback_signal_L, fclamp(feedback_signal_L, -1.0f, 1.0f), .001f);
        fonepole(feedback_signal_R, fclamp(feedback_signal_R, -1.0f, 1.0f), .001f);
        delay_l.Write(feedback_signal_L);
        delay_r.Write(feedback_signal_R);
        verb.Process(dryL + delay_out_L, dryR + delay_out_R, &wetL, &wetR);
        out[0][i] = (dryL * (1.0f - dr_mix)) + wetL;
        out[1][i] = (dryR * (1.0f - dr_mix)) + wetR;
        break;
      }
      case CHORUS_MODE: {
        wetL = chorus_l.Process(dryL);
        wetR = chorus_r.Process(dryR);
        out[0][i] = (dryL * (1.0f - chorus_mix)) + wetL;
        out[1][i] = (dryR * (1.0f - chorus_mix)) + wetR;
        break;
      }
    }
  }
}

// --- Arduino Setup ---
void setup() {
  patch = DAISY.init(DAISY_PATCH, AUDIO_SR_48K);
  samplerate = DAISY.get_samplerate();

  verb.Init(samplerate);
  delay_l.Init();
  delay_r.Init();
  chorus_l.Init(samplerate);
  chorus_r.Init(samplerate);
  // REMOVED incorrect SetLfoPhase call

  smoothed_delay_time = smoothed_dr_delay_time = samplerate * 0.5f;
  tap_delay_time_samples = smoothed_delay_time;

  clock_input.Init(A1, INPUT_PULLUP, true);
  button1.Init(samplerate, true, D1, INPUT_PULLUP);
  button2.Init(samplerate, true, D3, INPUT_PULLUP);

  current_effect = REVERB_MODE;
  DAISY.begin(AudioCallback);
}

// --- Arduino Loop ---
void loop() {
  patch.ProcessAllControls();
  button1.Debounce();
  button2.Debounce();

  EffectState old_effect = current_effect;
  int state_idx = (button1.Pressed() << 1) | button2.Pressed();
  current_effect = static_cast<EffectState>(state_idx);

  if(current_effect != old_effect) {
    switch (current_effect) {
      case REVERB_MODE:
        verb.Init(samplerate);
        break;
      case DELAY_MODE:
      case DELAY_REVERB_MODE:
        verb.Init(samplerate);
        delay_l.Reset();
        delay_r.Reset();
        break;
      case CHORUS_MODE:
        break;
    }
  }
  
  if (current_effect == REVERB_MODE) {
    fonepole(reverb_dry_level, CtrlVal(A5), 0.005f);
    fonepole(reverb_send, CtrlVal(A4), 0.005f);
    fonepole(reverb_feedback, 0.8f + CtrlVal(A3) * 0.199f, 0.005f);
    
    float reverb_adc_raw = analogRead(A0) / 65535.f;
    if(reverb_adc_raw < 0.05f) {
      fonepole(reverb_lp_freq, CtrlVal(A2) * 20000.0f, 0.005f);
    } else {
      float reverb_cv_scaled = ScaleCvLFO(reverb_adc_raw);
      fonepole(smoothed_reverb_cv, reverb_cv_scaled, 0.005f);
      float clamped_cv = fclamp(smoothed_reverb_cv, 0.02f, 0.98f);
      reverb_lp_freq = 400.f + (19600.f) * ((clamped_cv - 0.02f) / (0.96f));
    }
    verb.SetFeedback(reverb_feedback);
    verb.SetLpFreq(reverb_lp_freq);

  } else if (current_effect == DELAY_MODE) {
    fonepole(delay_mix, CtrlVal(A5), 0.005f);
    fonepole(delay_feedback, fmap(CtrlVal(A3), 0.f, 0.98f), 0.005f);
    fonepole(delay_send, fmap(CtrlVal(A2), 0.f, 0.8f), 0.005f);

    float clock_adc_raw = analogRead(A1) / 65535.f;
    if(clock_adc_raw < 0.05f) {
      delay_time = fmap(CtrlVal(A4), 10.f, 47999.f);
    } else {
        if(clock_input.Trig()) {
            ProcessTap();
        }
        delay_time = tap_delay_time_samples;
    }

  } else if (current_effect == DELAY_REVERB_MODE) {
    fonepole(dr_mix, CtrlVal(A5), 0.005f);
    fonepole(dr_delay_feedback, fmap(CtrlVal(A3), 0.f, 0.95f), 0.005f);
    fonepole(dr_reverb_decay, fmap(CtrlVal(A2), 0.6f, 0.98f), 0.005f);
    
    float clock_adc_raw = analogRead(A1) / 65535.f;
    if(clock_adc_raw < 0.05f) {
      dr_delay_time = fmap(CtrlVal(A4), 10.f, 47999.f);
    } else {
        if(clock_input.Trig()) {
            ProcessTap();
        }
        dr_delay_time = tap_delay_time_samples;
    }
    float reverb_adc_raw = analogRead(A0) / 65535.f;
    float current_lp_freq;
    if(reverb_adc_raw < 0.05f) {
      current_lp_freq = 10000.0f; 
    } else {
      float reverb_cv_scaled = ScaleCvLFO(reverb_adc_raw);
      fonepole(smoothed_reverb_cv, reverb_cv_scaled, 0.005f);
      float clamped_cv = fclamp(smoothed_reverb_cv, 0.02f, 0.98f);
      current_lp_freq = 400.f + (19600.f) * ((clamped_cv - 0.02f) / (0.96f));
    }
    verb.SetFeedback(dr_reverb_decay);
    verb.SetLpFreq(current_lp_freq);

  } else if (current_effect == CHORUS_MODE) {
    fonepole(chorus_mix, CtrlVal(A5), 0.005f);
    fonepole(chorus_depth, CtrlVal(A3), 0.005f);
    fonepole(chorus_feedback, CtrlVal(A2), 0.005f);

    float chorus_cv_raw = ScaleCvLFO(analogRead(A0) / 65535.f);
    if(chorus_cv_raw > 0.05f) {
      fonepole(chorus_rate, fmap(chorus_cv_raw, 0.1f, 10.f), 0.005f);
    } else {
      fonepole(chorus_rate, fmap(CtrlVal(A4), 0.1f, 10.f), 0.005f);
    }
    
    // CORRECTED: Set slightly different speeds for stereo width
    chorus_l.SetLfoFreq(chorus_rate);
    chorus_r.SetLfoFreq(chorus_rate * 1.05f);
    chorus_l.SetLfoDepth(chorus_depth);
    chorus_r.SetLfoDepth(chorus_depth);
    chorus_l.SetFeedback(chorus_feedback);
    chorus_r.SetFeedback(chorus_feedback);
  }
}