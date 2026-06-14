#include "plugin.hpp"
#include <algorithm>
#include <cmath>

// Array ceiling for the mass-spring ring. N (playable) tops out at 128; the
// arrays are sized larger so raising the N ceiling later needs no realloc.
static const int MAX_N = 256;

struct Haptik : Module {

    enum ParamId {
        N_PARAM,
        PITCH_PARAM,
        RATE_PARAM,
        COUPLE_PARAM,
        DAMP_PARAM,
        INJECT_PARAM,
        EXCITE_PARAM,
        FREEZE_PARAM,
        RATE_ATT_PARAM,
        COUPLE_ATT_PARAM,
        DAMP_ATT_PARAM,
        INJECT_ATT_PARAM,
        DRIVER_PARAM,        // appended last: keeps existing positional param IDs stable
        PARAMS_LEN
    };

    enum InputId {
        VOCT_INPUT,
        RATE_INPUT,
        COUPLE_INPUT,
        DAMP_INPUT,
        INJECT_INPUT,
        TRIG_INPUT,
        EXT_INPUT,
        INPUTS_LEN
    };

    enum OutputId {
        OUT_OUTPUT,
        MOTION_OUTPUT,
        OUTPUTS_LEN
    };

    enum LightId {
        LIGHTS_LEN
    };

    // ─── Persistent state ───────────────────────────────────────────────────
    float y[MAX_N] = {};            // displacement
    float v[MAX_N] = {};            // velocity
    float scanPhase = 0.f;          // [0,1) scan pointer
    int   last_N    = -1;           // -1 forces reinit on first process()
    int   driverIdx = 0;            // mass that excitation / EXT IN drives
    dsp::SchmittTrigger trig;
    dsp::TRCFilter<float> dcBlock;  // fixed internal DC blocker on OUT
    float lastFs = 0.f;             // detects SR change to refresh dcBlock cutoff

    // ─── Tunable constants ──────────────────────────────────────────────────
    static constexpr float DAMP_MAX_HZ = 800.f;
    static constexpr float OUT_GAIN    = 1.0f;
    static constexpr float MOTION_GAIN = 4.f;
    static constexpr float EXT_GAIN    = 0.005f;  // ±5V VCO at 0.2 saturated the ring; 0.005 lets it resonate
    static constexpr float BUMP_FRAC   = 0.125f;  // bump half-width = N * frac
    static constexpr float CV_DEPTH        = 0.1f;   // ±5V CV → ±0.5 at full attenuverter
    static constexpr float DRIVE_KEEPALIVE = 0.05f;  // continuous-drive noise amount (EXCITE=3)

    Haptik() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(N_PARAM, 8.f, 128.f, 64.f, "Masses")->snapEnabled = true;

        configParam(PITCH_PARAM, -4.f, 4.f, 0.f, "Pitch", " Hz", 2.f, dsp::FREQ_C4);
        configParam(RATE_PARAM, std::log2(0.05f), std::log2(30.f), std::log2(3.f),
                    "Evolution rate", " Hz", 2.f);
        configParam(COUPLE_PARAM, 0.f, 0.9f, 0.3f, "Coupling");
        configParam(DAMP_PARAM, 0.f, 1.f, 0.35f, "Damping");
        configParam(INJECT_PARAM, 0.f, 1.f, 0.6f, "Inject");

        configParam(EXCITE_PARAM, 0.f, 3.f, 1.f, "Excitation")->snapEnabled = true;

        configSwitch(FREEZE_PARAM, 0.f, 1.f, 0.f, "Freeze", {"Run", "Freeze"});

        configParam(RATE_ATT_PARAM, -1.f, 1.f, 0.f, "Rate CV");
        configParam(COUPLE_ATT_PARAM, -1.f, 1.f, 0.f, "Couple CV");
        configParam(DAMP_ATT_PARAM, -1.f, 1.f, 0.f, "Damp CV");
        configParam(INJECT_ATT_PARAM, -1.f, 1.f, 0.f, "Inject CV");

        // Driver position as a fraction of the ring; 0.25 == the old fixed N/4.
        configParam(DRIVER_PARAM, 0.f, 1.f, 0.25f, "Driver position", "%", 0.f, 100.f);

        configInput(VOCT_INPUT, "1V/oct pitch");
        configInput(RATE_INPUT, "Rate CV");
        configInput(COUPLE_INPUT, "Couple CV");
        configInput(DAMP_INPUT, "Damp CV");
        configInput(INJECT_INPUT, "Inject CV");
        configInput(TRIG_INPUT, "Trigger (re-excite)");
        configInput(EXT_INPUT, "External force");

        configOutput(OUT_OUTPUT, "Audio");
        configOutput(MOTION_OUTPUT, "Motion (driver-mass displacement, audio-rate CV)");
    }

    void onReset() override { last_N = -1; }   // force a clean lattice reseed on Initialize

    // Excitation writes displacement y[] (a "pluck"). amt is the inject amount.
    void applyExcite(int shape, float amt, int N) {
        switch (shape) {
            case 0:  // impulse: kick a single mass
                y[driverIdx] += amt;
                break;
            case 1: {  // bump: Hann window, half-width BUMP_FRAC*N, wraps mod N
                int hw = std::max(1, (int)(BUMP_FRAC * N));
                for (int d = -hw; d <= hw; d++) {
                    float w = 0.5f * (1.f + std::cos((float)M_PI * (float)d / (float)hw));
                    int idx = ((driverIdx + d) % N + N) % N;
                    y[idx] += amt * w;
                }
                break;
            }
            case 2:  // noise: perturb every mass
                for (int i = 0; i < N; i++)
                    y[i] += amt * (2.f * random::uniform() - 1.f);
                break;
            case 3:  // continuous drive: no impulse here (handled in kernel)
            default:
                break;
        }
    }

    void process(const ProcessArgs& args) override {
        const float fs = args.sampleRate;

        int N = (int) std::round(params[N_PARAM].getValue());
        N = clamp(N, 8, MAX_N);

        // Driver position is a live param (fraction of the ring), so recompute every
        // sample; set before reinit so the seed bump lands at the chosen mass.
        driverIdx = clamp((int)(params[DRIVER_PARAM].getValue() * N), 0, N - 1);

        // ── reinit on N change (also the first call) ──
        if (N != last_N) {
            std::fill(y, y + MAX_N, 0.f);
            std::fill(v, v + MAX_N, 0.f);
            scanPhase = 0.f;
            applyExcite(1, 1.f, N);            // always a full bump (ignores EXCITE/INJECT) so it sounds on load
            last_N = N;
        }

        // ── macros → physics ──
        // RATE_PARAM stores log2(Hz); CV adds in the log domain, att·CV_DEPTH scaled.
        float rateLog = params[RATE_PARAM].getValue()
                      + inputs[RATE_INPUT].getVoltage() * params[RATE_ATT_PARAM].getValue() * CV_DEPTH;
        float fEvo = clamp(std::exp2(rateLog), 0.05f, 30.f);
        float wc   = 2.f * (float)M_PI * fEvo / fs;
        float kCtr = wc * wc;   // centering is intentionally weak vs kSpr: RATE has
                                // subtle authority by design (audio-rate resonator)

        // Coupling clamped hard at 0.9 — this is the stability guarantee
        // (ω_max = sqrt(kCtr + 4·kSpr) < 2 for symplectic Euler). Do not raise.
        float kSpr = clamp(params[COUPLE_PARAM].getValue()
                         + inputs[COUPLE_INPUT].getVoltage() * params[COUPLE_ATT_PARAM].getValue() * CV_DEPTH,
                         0.f, 0.9f);

        float damp = clamp(params[DAMP_PARAM].getValue()
                         + inputs[DAMP_INPUT].getVoltage() * params[DAMP_ATT_PARAM].getValue() * CV_DEPTH,
                         0.f, 1.f);
        float gamma = std::exp(-damp * DAMP_MAX_HZ / fs);   // velocity multiplier ≤ 1

        float amt = clamp(params[INJECT_PARAM].getValue()
                        + inputs[INJECT_INPUT].getVoltage() * params[INJECT_ATT_PARAM].getValue() * CV_DEPTH,
                        0.f, 1.f);

        bool freeze = params[FREEZE_PARAM].getValue() > 0.5f;
        int  shape  = (int) std::round(params[EXCITE_PARAM].getValue());

        // ── excitation ──
        // Hysteresis window (0.1..1V) so an offset/DC-coupled trigger source can't latch.
        if (trig.process(inputs[TRIG_INPUT].getVoltage(), 0.1f, 1.f))
            applyExcite(shape, amt, N);

        float drive = inputs[EXT_INPUT].isConnected()
                    ? inputs[EXT_INPUT].getVoltage() * EXT_GAIN * amt : 0.f;
        if (shape == 3 && !freeze)
            drive += amt * DRIVE_KEEPALIVE * (2.f * random::uniform() - 1.f);   // continuous keep-alive

        // ── dynamics: symplectic Euler, two passes (skip if frozen) ──
        // Pass 1 reads only y[] (one snapshot) to form all accelerations;
        // pass 2 reads only v[]. The ordering is required for correctness.
        if (!freeze) {
            // Velocity pass. Interior masses need no index wrap; the two ring-seam
            // masses are handled out of the hot loop to keep it modulo-free.
            // +1e-20f flushes denormals on long DAMP=0 tails (inaudible, DC-blocked).
            float a0 = kSpr * (y[N - 1] - 2.f * y[0] + y[1]) - kCtr * y[0];
            v[0] = (v[0] + a0) * gamma + 1e-20f;
            for (int i = 1; i < N - 1; i++) {
                float a = kSpr * (y[i - 1] - 2.f * y[i] + y[i + 1]) - kCtr * y[i];
                v[i] = (v[i] + a) * gamma + 1e-20f;
            }
            float aN = kSpr * (y[N - 2] - 2.f * y[N - 1] + y[0]) - kCtr * y[N - 1];
            v[N - 1] = (v[N - 1] + aN) * gamma + 1e-20f;

            // Driver force folded in here: (v+a)·gamma + drive·gamma == (v+a+drive)·gamma.
            v[driverIdx] += drive * gamma;

            for (int i = 0; i < N; i++)
                y[i] += v[i];
        }

        // ── scan readout (always) ──
        float pitchHz = dsp::FREQ_C4 * std::exp2(
                            params[PITCH_PARAM].getValue() + inputs[VOCT_INPUT].getVoltage());
        scanPhase += pitchHz / fs;
        scanPhase -= std::floor(scanPhase);
        float p   = scanPhase * N;
        int   i0  = std::min((int) p, N - 1);    // guard against p == N at phase rounding
        float f   = p - i0;
        float s   = y[i0] + f * (y[(i0 + 1) % N] - y[i0]);

        // ── outputs ──
        if (fs != lastFs) {                      // recompute only on SR change (still
            dcBlock.setCutoffFreq(20.f / fs);    // graceful without onSampleRateChange)
            lastFs = fs;
        }
        dcBlock.process(s);                      // ~20 Hz high-pass; scanned mean wanders
        outputs[OUT_OUTPUT].setVoltage(5.f * std::tanh(dcBlock.highpass() * OUT_GAIN));
        // MOTION taps mass 0 directly (audio-rate in this design); not high-passed.
        outputs[MOTION_OUTPUT].setVoltage(clamp(y[0] * MOTION_GAIN, -5.f, 5.f));
    }
};


// ─── Widget ─────────────────────────────────────────────────────────────────

struct HaptikWidget : ModuleWidget {

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);

        std::shared_ptr<Font> font = APP->window->loadFont(
            asset::system("res/fonts/DejaVuSans.ttf"));
        if (!font) return;

        nvgSave(args.vg);
        nvgFontFaceId(args.vg, font->handle);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        const NVGcolor title  = nvgRGB(0xe0, 0xe0, 0xff);
        const NVGcolor dim    = nvgRGB(0x77, 0x77, 0x99);
        const NVGcolor outclr = nvgRGB(0xcc, 0xcc, 0xee);

        auto lbl = [&](float x, float y, float sz, NVGcolor col, const char* s) {
            nvgFontSize(args.vg, mm2px(sz));
            nvgFillColor(args.vg, col);
            nvgText(args.vg, mm2px(x), mm2px(y), s, nullptr);
        };

        // Title
        nvgFontSize(args.vg, mm2px(4.6f));
        nvgTextLetterSpacing(args.vg, mm2px(0.4f));
        nvgFillColor(args.vg, title);
        nvgText(args.vg, mm2px(30.48f), mm2px(8.f), "HAPTIK", nullptr);
        nvgTextLetterSpacing(args.vg, 0.f);

        // Top row labels
        lbl(10.f, 26.f, 1.9f, dim, "N");
        lbl(25.f, 26.f, 1.9f, dim, "PITCH");
        lbl(40.f, 26.f, 1.9f, dim, "EXCITE");
        lbl(52.f, 26.f, 1.9f, dim, "FREEZE");

        // CV-row column headers
        lbl(33.f, 33.f, 1.6f, dim, "att");
        lbl(48.f, 33.f, 1.6f, dim, "cv");

        // CV row labels (knob centre + 6 mm)
        lbl(12.f, 44.f, 1.9f, dim, "RATE");
        lbl(12.f, 58.f, 1.9f, dim, "COUPLE");
        lbl(12.f, 72.f, 1.9f, dim, "DAMP");
        lbl(12.f, 86.f, 1.9f, dim, "INJECT");

        // Input labels
        lbl(12.f, 103.f, 1.9f, dim, "V/OCT");
        lbl(30.48f, 103.f, 1.9f, dim, "TRIG");
        lbl(48.f, 103.f, 1.9f, dim, "EXT");

        // Driver-position label
        lbl(54.f, 114.f, 1.7f, dim, "DRIVER");

        // Output labels
        lbl(18.f, 120.f, 2.0f, outclr, "OUT");
        lbl(42.f, 120.f, 2.0f, outclr, "MOTION");

        nvgRestore(args.vg);
    }

    HaptikWidget(Haptik* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Haptik.svg")));

        addChild(createWidget<ScrewSilver>(mm2px(Vec(1.0f,   1.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(54.94f, 1.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(1.0f,   122.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(54.94f, 122.0f))));

        // Top row: N | PITCH | EXCITE | FREEZE
        addParam(createParamCentered<RoundBlackSnapKnob>(mm2px(Vec(10.f, 20.f)), module, Haptik::N_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(    mm2px(Vec(25.f, 20.f)), module, Haptik::PITCH_PARAM));
        addParam(createParamCentered<RoundBlackSnapKnob>(mm2px(Vec(40.f, 20.f)), module, Haptik::EXCITE_PARAM));
        addParam(createParamCentered<CKSS>(              mm2px(Vec(52.f, 20.f)), module, Haptik::FREEZE_PARAM));

        // CV rows: knob | att | cv
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.f, 38.f)), module, Haptik::RATE_PARAM));
        addParam(createParamCentered<Trimpot>(       mm2px(Vec(33.f, 38.f)), module, Haptik::RATE_ATT_PARAM));
        addInput(createInputCentered<PJ301MPort>(    mm2px(Vec(48.f, 38.f)), module, Haptik::RATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.f, 52.f)), module, Haptik::COUPLE_PARAM));
        addParam(createParamCentered<Trimpot>(       mm2px(Vec(33.f, 52.f)), module, Haptik::COUPLE_ATT_PARAM));
        addInput(createInputCentered<PJ301MPort>(    mm2px(Vec(48.f, 52.f)), module, Haptik::COUPLE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.f, 66.f)), module, Haptik::DAMP_PARAM));
        addParam(createParamCentered<Trimpot>(       mm2px(Vec(33.f, 66.f)), module, Haptik::DAMP_ATT_PARAM));
        addInput(createInputCentered<PJ301MPort>(    mm2px(Vec(48.f, 66.f)), module, Haptik::DAMP_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.f, 80.f)), module, Haptik::INJECT_PARAM));
        addParam(createParamCentered<Trimpot>(       mm2px(Vec(33.f, 80.f)), module, Haptik::INJECT_ATT_PARAM));
        addInput(createInputCentered<PJ301MPort>(    mm2px(Vec(48.f, 80.f)), module, Haptik::INJECT_INPUT));

        // Inputs: V/OCT | TRIG | EXT
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.f,    97.f)), module, Haptik::VOCT_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48f, 97.f)), module, Haptik::TRIG_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(48.f,    97.f)), module, Haptik::EXT_INPUT));

        // Driver position (lower-right, by the excitation inputs)
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(54.f, 108.f)), module, Haptik::DRIVER_PARAM));

        // Outputs: OUT | MOTION
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.f, 114.f)), module, Haptik::OUT_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(42.f, 114.f)), module, Haptik::MOTION_OUTPUT));
    }
};

Model* modelHaptik = createModel<Haptik, HaptikWidget>("Haptik");
