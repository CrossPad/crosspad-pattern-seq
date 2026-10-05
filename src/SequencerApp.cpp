/**
 * @file SequencerApp.cpp
 * @brief Sequencer -- the loop recorder, on a screen of its own.
 *
 * Everything that used to slide in from the Sampler's "Advanced" strip: tempo,
 * click, the scene's length and grid, x2 and /2, the scene picker, held ERASE
 * and the transport -- around a view of what is recorded. The pattern is drawn
 * as sixteen lanes, one per pad, across the sounding scene's length; every
 * note a tick in its pad's colour, brighter the harder it was hit, and a
 * playhead running over them. Hold ERASE, hit a pad, and its lane empties.
 *
 * BOUNCE plays the sounding scene once, from its bar one, and writes its pads
 * to a clip in the open song project, on the track picked -- where Arrange
 * lays it out with the others.
 *
 * The pads stay the Sampler's: no pad logic here, so they play the kit and
 * record exactly as they do from any other screen. The loop keeps running
 * when the app closes; it lives in the sequencer, not in this screen.
 */

#include "crosspad-sequencer/SequencerApp.hpp"

// `class App` lives in a different header per platform, but both derive from
// crosspad_gui::AppBase<App>, so the lifecycle calls used here are identical.
#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/instrument/PitchedInstrument.hpp"
#include "crosspad/music/MusicTheory.hpp"
#include "crosspad/kit/IKitManager.hpp"
#include "crosspad/kit/KitInfo.hpp"
#include "crosspad/pad/IPadLogicHandler.hpp"
#include "crosspad/pad/PadManager.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad/sequencer/SequencerController.hpp"
#include "crosspad/song/ISongEngine.hpp"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"
/* lv_image_cache_drop(): lvgl.h stopped exporting the cache in 9.4. */
#include "src/misc/cache/instance/lv_image_cache.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Encoder navigation is an embedded-only concern -- the simulator drives the UI
// with a pointer. Snapshot whichever group was active before the app opened and
// restore that, rather than a global launcher group that goes stale whenever the
// launcher is rebuilt.
#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define SEQ_USE_ENCODER 1
#endif

namespace {

/* ── set_text_fmt() is not used here, and this is why ─────────────
 *
 * It frees the label's old text and allocates the new one, but it does not
 * revert the dots of a truncated label first -- lv_label_set_text() does
 * exactly that, one line in, "in case text == label->text". So when a long
 * line is replaced by a shorter one, LVGL still holds the offset the dots were
 * written at, and on the next refresh it puts the four saved characters back at
 * that offset -- past the end of the new, shorter buffer. The heap block next
 * door loses its tail canary, and the crash lands minutes later on whatever
 * frees that block: an assert deep inside LVGL, with nothing in the backtrace
 * to say who wrote the bytes. It read as "Arrange crashes when I click a
 * block", because this screen is the one that formats a line of changing length
 * on every click.
 *
 * Formatting it ourselves and handing lv_label_set_text() a finished string is
 * both the safe way and what every other screen in this firmware does. */
static void set_text_fmt(lv_obj_t* label, const char* fmt, ...)
{
    if (!label || !fmt) return;
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lv_label_set_text(label, buf);
}

constexpr uint32_t kTickMs = 50;
constexpr int      kLanes  = 16;

const lv_color_t kBg     = lv_color_hex(0x16181B);
const lv_color_t kPanel  = lv_color_hex(0x20242A);
const lv_color_t kLine   = lv_color_hex(0x2C3138);
const lv_color_t kBarLn  = lv_color_hex(0x4A515B);
const lv_color_t kText   = lv_color_hex(0xE8EAED);
const lv_color_t kDim    = lv_color_hex(0x7B828B);
const lv_color_t kInk    = lv_color_hex(0x16181B);
const lv_color_t kAccent = lv_color_hex(0xC6FF47);
const lv_color_t kRec    = lv_color_hex(0xFF5C5C);
const lv_color_t kPlay   = lv_color_hex(0x4FE0B0);
const lv_color_t kAmber  = lv_color_hex(0xFFC857);
const lv_color_t kHead   = lv_color_hex(0xFFFFFF);
/// The tempo field while a host's MIDI clock drives the tempo.
const lv_color_t kHostClock = lv_color_hex(0x4FC3F7);

lv_obj_t* s_root = nullptr;
lv_obj_t* s_grid = nullptr;
lv_obj_t* s_bpm = nullptr;
lv_obj_t* s_syncBtn = nullptr;
lv_obj_t* s_clickBtn = nullptr;
lv_obj_t* s_preBtn = nullptr;
lv_obj_t* s_barsLabel = nullptr;
lv_obj_t* s_quantBtn = nullptr;
lv_obj_t* s_quantLabel = nullptr;
lv_obj_t* s_sceneLabel = nullptr;
lv_obj_t* s_eraseBtn = nullptr;
lv_obj_t* s_recBtn = nullptr;
lv_obj_t* s_dubBtn = nullptr;
lv_obj_t* s_playBtn = nullptr;
lv_obj_t* s_stopBtn = nullptr;
lv_timer_t* s_timer = nullptr;
/* Subscribed on the first open and kept: the clock publishes only changes, so
 * a screen opened while the host already drives must find the flag current. */
crosspad::SubscriptionHandle s_tempoSub = nullptr;
std::atomic<bool> s_tempoFromHost{false};

/* The bounce window: track picker, then progress, then the result. */
lv_obj_t* s_bounceBox = nullptr;
lv_obj_t* s_bounceText = nullptr;
lv_obj_t* s_bounceBar = nullptr;
lv_obj_t* s_bounceBtn = nullptr;     // Cancel while running, OK after
bool      s_bouncing = false;

/* What the grid was last drawn from; redrawn whole when it changes. The
 * playhead is the only thing that moves while a scene plays, and only its
 * two columns (where it was, where it is) are redrawn for it: the whole grid
 * every tick kept the LVGL task busy most of the time on the core the card
 * writers share. */
uint32_t s_drawn = 0;
int32_t  s_headX = -1;   // screen x of the playhead as drawn, -1 = none

#ifdef SEQ_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_bounceGroup = nullptr;
lv_obj_t*   s_swingBox  = nullptr;      ///< the swing window, over the screen
lv_obj_t*   s_swingText = nullptr;      ///< what it is set to, in words
lv_obj_t*   s_swingVal  = nullptr;      ///< the amount, turned with the encoder
lv_obj_t*   s_swingChip[7] = {};        ///< the sampler's steps, 50 to 75
lv_obj_t*   s_swingBtn  = nullptr;      ///< SWING on the top row

/* SRC: what the pads play, and the window that picks it. */
lv_obj_t* s_srcBtn   = nullptr;         ///< SRC on the transport row
lv_obj_t* s_srcLabel = nullptr;         ///< its text: KIT, or the instrument
lv_obj_t* s_srcBox   = nullptr;         ///< the picker, over the screen
lv_obj_t* s_srcList  = nullptr;         ///< the scrolling part of it
lv_obj_t* s_srcNoteLbl = nullptr;       ///< a failure or "loading", under the list
lv_obj_t* s_srcOctLbl  = nullptr;       ///< the base note
lv_obj_t* s_srcChordLbl = nullptr;      ///< what a pad makes: one note, or a chord
lv_obj_t* s_srcChordMinus = nullptr;
lv_obj_t* s_srcChordPlus = nullptr;
lv_obj_t* s_srcMinus = nullptr;         ///< the three below the list; the group
lv_obj_t* s_srcPlus  = nullptr;         ///< is rebuilt around them on every
lv_obj_t* s_srcClose = nullptr;         ///< fill, so they keep their order
lv_group_t* s_srcGroup = nullptr;
std::vector<std::string> s_srcLib;      ///< instrument folder names
bool s_srcListing = false;              ///< a folder request of ours is out
lv_group_t* s_swingGroup = nullptr;
#endif

crosspad::SequencerController* seq()
{
    return crosspad::getPlatformServices().sequencer;
}

crosspad::KitInfo* current_kit()
{
    auto* km = crosspad::getKitManager();
    return km ? km->getCurrentKit() : nullptr;
}

bool running()
{
    auto* sq = seq();
    return sq && sq->getState() == crosspad::SequencerState::Running;
}

// ── what the pads play, and so what a take records ─────────────────────────

/* The Sequencer has no sound of its own: it records whatever the pads make,
 * and a bounce captures the pads bus (SongEngine::runBounce), so whichever of
 * these is playing is what lands on the track. Until now that was always the
 * kit. SRC also offers the instruments Pitch Play saves, so a melodic part can
 * be recorded and bounced here rather than played by hand into the Sampler.
 *
 * The pitched engine and the kits' streamer share one arena, so an instrument
 * is *borrowed* through IInstrumentHost: taking it stops the kits' engine,
 * giving it back reloads the kit. Hence release() on the way out -- leave it
 * borrowed and the Sampler comes back silent. The one exception is a bounce
 * still running when the screen closes: it is capturing the pads, so the
 * engine has to stay until it is done.
 *
 * The pads need a logic of their own in this mode (the Sampler's plays kit
 * pads, not notes), pushed on the way in and popped on the way out. This app
 * registers with plain REGISTER_APP and no padLogicName, so the push is ours
 * to make -- an app declaring one must not. */

constexpr uint8_t kSeqPads = 16;
constexpr int      kSeqChordMax = 6;    ///< the widest voicing MusicTheory builds
constexpr const char* kSrcPadLogic = "SeqPitch";

/* A pad plays a chord, not only its own note. The mapping is
 * crosspad::MusicTheory's -- the same one Pitch Play plays through, so a
 * triad here is the triad there -- over a chromatic scale rooted on the pad
 * range's base: the Sequencer has an octave control and no key of its own,
 * and inventing a second place to set one would be two to keep in step.
 *
 * In PSRAM: PlaySettings is ~150 bytes and this board boots with a couple of
 * hundred to spare. */
#ifdef ESP_PLATFORM
  #include "esp_attr.h"
  #define SEQ_SLOW_BSS EXT_RAM_BSS_ATTR
#else
  #define SEQ_SLOW_BSS
#endif

SEQ_SLOW_BSS crosspad::MusicTheory  s_mt;
SEQ_SLOW_BSS crosspad::PlaySettings s_ps;
SEQ_SLOW_BSS int s_chordIdx;            ///< 0 = one note, then the chord types and progressions

enum class Src : uint8_t { Kit, Inst };

Src         s_src = Src::Kit;
std::string s_instPath;                  ///< logical path of the chosen instrument
std::string s_instName;                  ///< what SRC and the window show
std::string s_srcNote;                   ///< a failure or a progress line, for the window
uint8_t     s_base = 60;                 ///< C4: the 16 pads are 16 semitones up from here

void seq_rebuild_pads()
{
    s_ps.rootNote   = (uint8_t)(s_base % 12);
    s_ps.octave     = (int8_t)(s_base / 12 - 3);
    s_ps.scaleIndex = 0;                 // chromatic: one pad, one semitone
    s_ps.chordIndex = (uint8_t)s_chordIdx;
    s_mt.rebuildPadMap(s_ps);
}

/// The notes a press on @p pad makes. At least one, at most kSeqChordMax.
int seq_chord_of(uint8_t pad, uint8_t* out)
{
    if (pad >= kSeqPads) { out[0] = 0; return 1; }
    int n = 0;
    const int have = s_ps.padChordCount[pad];
    for (int c = 0; c < have && c < kSeqChordMax; ++c) {
        if (s_ps.padChordNotes[pad][c] != 0xFF) out[n++] = s_ps.padChordNotes[pad][c];
    }
    if (n == 0) out[n++] = s_ps.padMidiMap[pad];
    return n;
}
std::atomic<bool> s_instReady{false};    ///< the bank is built and the pads may play it
bool        s_padLogicPushed = false;

crosspad::IInstrumentHost*   inst_host() { return crosspad::getPlatformServices().instrumentHost; }
crosspad::PitchedInstrument* inst()      { return crosspad::getPlatformServices().pitchedInstrument; }
crosspad::ISampleTools*      tools()     { return crosspad::getPlatformServices().sampleTools; }

void note_name(int note, char* buf, size_t n)
{
    static const char* kNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    if (note < 0 || note > 127) { std::snprintf(buf, n, "--"); return; }
    std::snprintf(buf, n, "%s%d", kNames[note % 12], note / 12 - 1);
}

/// Pads as sixteen semitones from s_base. The note a pad started on is kept,
/// so a pad released after the base moved still stops what it began.
class SeqPitchPadLogic : public crosspad::IPadLogicHandler {
public:
    void onActivate(crosspad::PadManager&) override {}
    void onDeactivate(crosspad::PadManager&) override
    {
        if (auto* p = inst()) p->allNotesOff();
    }
    void onPadPress(crosspad::PadManager& pm, uint8_t pad, uint8_t vel) override
    {
        if (pad >= kSeqPads || !s_instReady.load()) return;
        auto* p = inst();
        if (!p) return;
        /* Kept as struck, so a release stops those notes and not whatever the
         * octave or the chord says by then. */
        uint8_t notes[kSeqChordMax] = {};
        const int n = seq_chord_of(pad, notes);
        held_[pad].store((uint8_t)n);
        for (int i = 0; i < n; ++i) {
            note_[pad][i].store(notes[i]);
            p->noteOn(notes[i], vel ? vel : 1, pad);
        }
        pm.notifyNoteOn(pad, vel ? vel : 1, true);
    }
    void onPadRelease(crosspad::PadManager& pm, uint8_t pad) override
    {
        if (pad >= kSeqPads) return;
        auto* p = inst();
        const int n = (int)held_[pad].exchange(0);
        for (int i = 0; i < n; ++i) if (p) p->noteOff(note_[pad][i].load());
        pm.notifyNoteOff(pad, true);
    }
    void onPadPressure(crosspad::PadManager&, uint8_t, uint8_t) override {}

private:
    std::atomic<uint8_t> note_[kSeqPads][kSeqChordMax] = {};
    std::atomic<uint8_t> held_[kSeqPads] = {};
};
std::shared_ptr<SeqPitchPadLogic> s_srcPadLogic;

// ── pattern grid ───────────────────────────────────────────────────────────

void fill(lv_layer_t* layer, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
          lv_color_t c, lv_opa_t opa)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = opa;
    d.radius = 0;
    lv_area_t a = {x1, y1, x2, y2};
    lv_draw_rect(layer, &d, &a);
}

/// Drawn straight into the layer rather than built from objects: a full scene
/// is 192 notes, and 192 LVGL objects rebuilt on every hit would cost more
/// than the pattern is worth.
void grid_draw_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_layer_t* layer = lv_event_get_layer(e);
    lv_obj_t* obj = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (!sq || !layer) return;

    lv_area_t box;
    lv_obj_get_content_coords(obj, &box);
    const int32_t w = lv_area_get_width(&box);
    const int32_t h = lv_area_get_height(&box);
    const int32_t laneH = h / kLanes;
    const int32_t x0 = box.x1 + 4;          // after the pad-colour strip
    const int32_t gw = w - 4;
    if (laneH < 2 || gw < 16) return;

    const uint8_t scene = sq->getActiveScene();
    const int bars = sq->getSceneBars(scene) ? sq->getSceneBars(scene) : 1;
    const int beats = bars * 4;
    crosspad::KitInfo* kit = current_kit();

    /* Lanes: the pad's own colour as a strip at the left edge, so a note's
     * colour can be matched to its row without a label. Pad 1 at the bottom,
     * the way the pads sit under your hand. */
    auto laneY = [&](int pad) { return box.y1 + (kLanes - 1 - pad) * laneH; };
    for (int p = 0; p < kLanes; ++p) {
        const int32_t y = laneY(p);
        const lv_color_t pc = kit ? lv_color_hex(kit->pads[p].color) : kDim;
        fill(layer, box.x1, y, box.x1 + 2, y + laneH - 2, pc, LV_OPA_COVER);
        if (p % 4 == 0 && p > 0) {      // under a row of four: the line to the row below
            fill(layer, x0, y + laneH - 1, x0 + gw - 1, y + laneH - 1, kLine, LV_OPA_COVER);
        }
    }
    /* Beats faint, bars strong: the grid a take is snapped to. */
    for (int b = 0; b <= beats; ++b) {
        const int32_t x = x0 + (int32_t)((int64_t)gw * b / beats);
        fill(layer, x, box.y1, x, box.y1 + laneH * kLanes - 1,
             b % 4 == 0 ? kBarLn : kLine, LV_OPA_COVER);
    }

    const int32_t noteW = gw / (beats * 4) > 2 ? gw / (beats * 4) - 1 : 2;
    const uint16_t n = sq->getSceneEventCount(scene);
    for (uint16_t i = 0; i < n; ++i) {
        float at = 0.0f;
        uint8_t pad = 0, vel = 0;
        if (!sq->getSceneEvent(scene, i, at, pad, vel) || pad >= kLanes) continue;
        const int32_t x = x0 + (int32_t)(at * gw);
        const int32_t y = laneY(pad);
        const lv_color_t pc = kit ? lv_color_hex(kit->pads[pad].color) : kAccent;
        const lv_opa_t opa = (lv_opa_t)(110 + (vel > 127 ? 127 : vel) * 145 / 127);
        fill(layer, x, y, x + noteW - 1, y + laneH - 2, pc, opa);
    }

    if (s_headX >= 0) fill(layer, s_headX, box.y1, s_headX + 1, box.y1 + laneH * kLanes - 1, kHead, LV_OPA_COVER);
}

/// Where the playhead belongs now, in screen x; -1 when stopped.
int32_t head_x()
{
    auto* sq = seq();
    if (!sq || !s_grid || !running()) return -1;
    lv_area_t box;
    lv_obj_get_content_coords(s_grid, &box);
    const int32_t gw = lv_area_get_width(&box) - 4;
    return box.x1 + 4 + (int32_t)(sq->getProgress() * gw);
}

void invalidate_head(int32_t x)
{
    if (x < 0 || !s_grid) return;
    lv_area_t box;
    lv_obj_get_content_coords(s_grid, &box);
    lv_area_t a = {x, box.y1, x + 1, box.y2};
    lv_obj_invalidate_area(s_grid, &a);
}

/// Everything the grid shows that is not the playhead.
uint32_t pattern_shape()
{
    auto* sq = seq();
    if (!sq) return 0;
    const uint8_t s = sq->getActiveScene();
    return ((uint32_t)s << 24) ^ ((uint32_t)sq->getSceneBars(s) << 16) ^
           sq->getSceneEventCount(s) ^ ((uint32_t)sq->getSceneCount() << 20);
}

// ── state on the buttons ───────────────────────────────────────────────────

/* Only what changed: setting a style, even to the value it has, refreshes the
 * object and redraws it, and sync_ui() runs every tick. */
void paint(lv_obj_t* b, lv_color_t bg, bool dark)
{
    if (!b) return;
    if (!lv_color_eq(lv_obj_get_style_bg_color(b, LV_PART_MAIN), bg)) lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    const lv_color_t ink = dark ? kInk : kText;
    if (l && !lv_color_eq(lv_obj_get_style_text_color(l, LV_PART_MAIN), ink)) lv_obj_set_style_text_color(l, ink, 0);
}

void set_text(lv_obj_t* l, const char* text)
{
    if (l && std::strcmp(lv_label_get_text(l), text) != 0) lv_label_set_text(l, text);
}

void sync_ui()
{
    auto* sq = seq();
    if (!sq) return;

    const bool run = running();
    const auto mode = sq->getMode();
    const bool rec = run && mode == crosspad::SequencerMode::Record;
    const bool dub = run && mode == crosspad::SequencerMode::Overdub;
    paint(s_recBtn,  rec ? kRec : kPanel, rec);
    paint(s_dubBtn,  dub ? kRec : kPanel, dub);

    /* Counting in: the take's button counts down the beats left, amber, and
     * turns red on its beat one. */
    const uint8_t left = sq->getCountInLeft();
    lv_obj_t* waiting = !left ? nullptr
                      : (mode == crosspad::SequencerMode::Overdub ? s_dubBtn : s_recBtn);
    char buf[24];
    if (waiting) {
        paint(waiting, kAmber, true);
        std::snprintf(buf, sizeof(buf), "%u", (unsigned)left);
        set_text(lv_obj_get_child(waiting, 0), buf);
    }
    if (waiting != s_recBtn) set_text(lv_obj_get_child(s_recBtn, 0), "REC");
    if (waiting != s_dubBtn) set_text(lv_obj_get_child(s_dubBtn, 0), "DUB");
    paint(s_preBtn, sq->getCountIn() ? kAmber : kPanel, sq->getCountIn());
    /* The amount on the button: a feel you cannot see the value of is one you
     * cannot put back where it was. */
    if (s_swingBtn) {
        const uint8_t sw = sq->getSwing();
        paint(s_swingBtn, sw ? kAmber : kPanel, sw != 0);
        char sb[12];
        if (sw) std::snprintf(sb, sizeof(sb), "SW %u", (unsigned)sw);
        else    std::snprintf(sb, sizeof(sb), "SWING");
        set_text(lv_obj_get_child(s_swingBtn, 0), sb);
    }
    paint(s_playBtn, (run && !rec && !dub) ? kPlay : kPanel, run && !rec && !dub);
    paint(s_stopBtn, kPanel, false);
    paint(s_clickBtn, sq->getClickActive() ? kAmber : kPanel, sq->getClickActive());
    if (auto* se = crosspad::getPlatformServices().songEngine) {
        bool sync = false;
        {
            std::lock_guard<std::recursive_mutex> l(se->songMutex());
            sync = se->song().syncSeq;
        }
        paint(s_syncBtn, sync ? kAccent : kPanel, sync);
    }
    if (!sq->isErasing()) paint(s_eraseBtn, kPanel, false);

    const bool off = sq->getQuantization() == crosspad::Quantization::Off;
    paint(s_quantBtn, off ? kAmber : kPanel, off);
    const char* q = "1/16";
    switch (sq->getQuantization()) {
        case crosspad::Quantization::Quarter:          q = "1/4";   break;
        case crosspad::Quantization::Eighth:           q = "1/8";   break;
        case crosspad::Quantization::EighthTriplet:    q = "1/8T";  break;
        case crosspad::Quantization::SixteenthTriplet: q = "1/16T"; break;
        case crosspad::Quantization::ThirtySecond:     q = "1/32";  break;
        case crosspad::Quantization::Off:              q = "free";  break;
        default: break;
    }
    set_text(s_quantLabel, q);

    const uint8_t active = sq->getActiveScene();
    const uint8_t pending = sq->getPendingScene();
    const uint8_t bars = sq->getSceneBars(active) ? sq->getSceneBars(active) : 1;
    std::snprintf(buf, sizeof(buf), "%ub", (unsigned)bars);
    set_text(s_barsLabel, buf);

    /* The scene you asked for, amber until the loop end hands it over. */
    if (pending != active) {
        std::snprintf(buf, sizeof(buf), "Seq %u > %u", (unsigned)(active + 1), (unsigned)(pending + 1));
    } else {
        std::snprintf(buf, sizeof(buf), "Seq %u / %u", (unsigned)(active + 1), (unsigned)sq->getSceneCount());
    }
    set_text(s_sceneLabel, buf);
    const lv_color_t sceneInk = pending != active ? kAmber : kText;
    if (!lv_color_eq(lv_obj_get_style_text_color(s_sceneLabel, LV_PART_MAIN), sceneInk)) {
        lv_obj_set_style_text_color(s_sceneLabel, sceneInk, 0);
    }

    if (s_bpm) {
        std::snprintf(buf, sizeof(buf), "%03d", (int)lroundf(sq->getBpm()));
        set_text(lv_obj_get_child(s_bpm, 0), buf);
        /* Blue while a host's MIDI clock drives the tempo. */
        const lv_color_t field = s_tempoFromHost.load(std::memory_order_relaxed) ? kHostClock : kPanel;
        if (!lv_color_eq(lv_obj_get_style_bg_color(s_bpm, LV_PART_MAIN), field)) {
            lv_obj_set_style_bg_color(s_bpm, field, 0);
        }
    }
}

void bounce_progress();
void src_poll();

void tick_cb(lv_timer_t*)
{
    if (s_bouncing) bounce_progress();
    src_poll();
    sync_ui();
    const uint32_t shape = pattern_shape();
    const int32_t x = head_x();
    if (shape != s_drawn) {
        s_drawn = shape;
        s_headX = x;
        if (s_grid) lv_obj_invalidate(s_grid);
    } else if (x != s_headX) {
        invalidate_head(s_headX);
        s_headX = x;
        invalidate_head(s_headX);
    }
}

// ── actions ────────────────────────────────────────────────────────────────

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

/// The tempo goes to the clock at once -- the pattern is rescaled with it, so
/// a take keeps its place in the bar -- and sequences.json carries it with
/// the scenes. It is the groove's, not the kit's: another kit plays it as is.
void bpm_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    if (!sq || !b) return;
    lv_group_t* g = lv_obj_get_group(b);
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        /* The encoder's second click, or a tap: leave edit mode, or enter it. */
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int step = key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
    /* The DAW's clock owns the tempo while it drives: a turn changes
     * nothing, and sequences.json is left alone. */
    if (!step || s_tempoFromHost.load(std::memory_order_relaxed)) return;
    const int32_t v = std::max<int32_t>((int32_t)crosspad::SequencerLimits::MIN_BPM,
                      std::min<int32_t>((int32_t)crosspad::SequencerLimits::MAX_BPM,
                                        (int32_t)lroundf(sq->getBpm()) + step));
    sq->setBpm((float)v);
    sync_ui();
}

/// A bar of click before a take started from a stop, so the first hit lands
/// on beat one instead of on the button press.
void pre_cb(lv_event_t*)
{
    if (auto* sq = seq()) sq->setCountIn(!sq->getCountIn());
    sync_ui();
}

void click_cb(lv_event_t*)
{
    if (auto* sq = seq()) sq->setClickActive(!sq->getClickActive());
    sync_ui();
}

void bars_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t s = sq->getActiveScene();
    const uint8_t bars = sq->getSceneBars(s);
    sq->setSceneBars(s, (bars >= 8) ? 1 : (uint8_t)(bars ? bars * 2 : 1));
    sync_ui();
}

/// 1/4, 1/8, 1/8T, 1/16, 1/16T, 1/32, free, and round again. The triplets sit
/// where they belong by density -- three to the beat is between two and four --
/// so one way round the button is coarse to fine and nothing jumps back. Free
/// stands one press past the finest, because a part with feel in it is ruined by
/// the same snap that flatters most playing.
void quant_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    using Q = crosspad::Quantization;
    static const Q kRound[] = {Q::Quarter, Q::Eighth, Q::EighthTriplet, Q::Sixteenth,
                               Q::SixteenthTriplet, Q::ThirtySecond, Q::Off};
    constexpr size_t kCount = sizeof(kRound) / sizeof(kRound[0]);
    const Q now = sq->getQuantization();
    size_t i = 0;
    while (i < kCount && kRound[i] != now) ++i;
    sq->setQuantization(kRound[i + 1 < kCount ? i + 1 : 0]);   // nothing known: back to 1/4
    sync_ui();
}

void double_cb(lv_event_t*) { if (auto* sq = seq()) sq->doubleScene(sq->getActiveScene()); sync_ui(); }
void halve_cb(lv_event_t*)  { if (auto* sq = seq()) sq->halveScene(sq->getActiveScene()); sync_ui(); }

void scene_step(int d)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t n = sq->getSceneCount();
    if (!n) return;
    sq->selectScene((uint8_t)((sq->getPendingScene() + n + d) % n));
    sync_ui();
}
void scene_prev_cb(lv_event_t*) { scene_step(-1); }
void scene_next_cb(lv_event_t*) { scene_step(+1); }

/// Minus drops the scene on show, plus appends an empty one and goes to it.
/// The last scene never goes -- a kit has to have somewhere to record.
void scene_del_cb(lv_event_t*) { if (auto* sq = seq()) sq->removeScene(sq->getActiveScene()); sync_ui(); }
void scene_add_cb(lv_event_t*) { if (auto* sq = seq()) sq->addScene(); sync_ui(); }

/// Held, not toggled: hold it (the encoder's button or a finger) and hit
/// pads -- each pad hit leaves the sounding scene. Letting go ends it.
void erase_cb(lv_event_t* e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;
    const bool on = code == LV_EVENT_PRESSED;
    if (auto* sq = seq()) sq->setErasing(on);
    paint(s_eraseBtn, on ? kRec : kPanel, on);
}

// ── transport, alone or with the song ──────────────────────────────────────

crosspad::ISongEngine* songs() { return crosspad::getPlatformServices().songEngine; }

/// SYNC on and a song to play: the transport starts the song, and the song
/// starts the sequencer with it (Arrange's SYNC is the same switch).
bool song_sync()
{
    auto* se = songs();
    if (!se) return false;
    std::lock_guard<std::recursive_mutex> l(se->songMutex());
    return se->song().syncSeq && se->song().playEnd() > 0;
}

void sync_cb(lv_event_t*)
{
    auto* se = songs();
    if (!se) return;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        se->song().syncSeq = !se->song().syncSeq;
    }
    se->commit();
    sync_ui();
}

/// From a stop, in sync: the song from its top, the sequencer doing @p take.
///
/// PRE counts here exactly as it does without the song. The count-in belongs to
/// the take, and in sync it is the metronome that plays it -- the song and the
/// scene both start on the sample after the last click. Until this asked for it
/// the switch quietly dropped PRE: a take started from here goes in through the
/// song's transport, and only a take started on its own ever read the sequencer
/// and counted. Plain PLAY is still immediate, with the song or without it,
/// which is what PRE has always meant on this screen.
bool start_with_song(crosspad::SyncTake take)
{
    auto* se = songs();
    if (!se || running() || !song_sync()) return false;
    auto* sq = seq();
    const uint8_t beats = (take != crosspad::SyncTake::None && sq) ? sq->getCountInBeats() : 0;
    return beats ? se->playCountedIn(0, take, beats) : se->playWithTake(0, take);
}

/// A second REC closes the take without stopping the loop, which is what you
/// want after playing the part in.
void rec_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    if (sq->isRecording())                                  sq->playback();
    else if (!start_with_song(crosspad::SyncTake::Record))  sq->record();
    sync_ui();
}
void dub_cb(lv_event_t*)
{
    auto* sq = seq();
    if (sq && !start_with_song(crosspad::SyncTake::Overdub)) sq->overdub();
    sync_ui();
}
void play_cb(lv_event_t*)
{
    auto* sq = seq();
    if (sq && !start_with_song(crosspad::SyncTake::None)) sq->playback();
    sync_ui();
}
/// The song too, when it is the one running the sequencer.
void stop_cb(lv_event_t*)
{
    auto* se = songs();
    if (se && se->playing() && song_sync()) se->stop();
    if (auto* sq = seq()) sq->stop();
    sync_ui();
}

// ── bounce ─────────────────────────────────────────────────────────────────

void bounce_close()
{
    if (!s_bounceBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_bounceGroup);          // its OK/Cancel click is still being read
    s_bounceGroup = nullptr;
#endif
    lv_obj_delete_async(s_bounceBox);
    s_bounceBox = s_bounceText = s_bounceBar = s_bounceBtn = nullptr;
    s_bouncing = false;
}

void bounce_close_cb(lv_event_t*)
{
    if (s_bouncing) {
        if (auto* se = songs()) se->cancelBounce();   // the result closes it
        return;
    }
    bounce_close();
}

/* Both window makers hang callbacks that are written below them, and they use
 * the row helper the main screen is built with. */
void swing_val_cb(lv_event_t* e);
void swing_set_cb(lv_event_t* e);
lv_obj_t* make_row(lv_obj_t* parent, int32_t h);

/// The amount, as a button the encoder turns: the idiom the tempo uses, so the
/// gesture is the one already learned on the screen behind.
lv_obj_t* swing_value(lv_obj_t* parent)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    lv_obj_set_size(b, 56, 26);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, swing_val_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(b, swing_val_cb, LV_EVENT_CLICKED, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
}

/// One of the sampler's steps: a chip in a row, not a row of its own -- seven
/// of them would not fit the screen and a step is one number.
lv_obj_t* swing_chip(lv_obj_t* parent, const char* text, void* intensity)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, 40, 24);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 5, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, swing_set_cb, LV_EVENT_CLICKED, intensity);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
}

/// A row of the swing window. Its own maker rather than the bounce one, which
/// adds itself to that window's group.
lv_obj_t* swing_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
}

lv_obj_t* bounce_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 26);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_add_obj(s_bounceGroup, b);
#endif
    return b;
}

/// Where the take is going, then how far it got.
void bounce_track_cb(lv_event_t* e)
{
    auto* se = songs();
    auto* sq = seq();
    if (!se || !sq || !s_bounceBox) return;
    const uint8_t track = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    const uint8_t scene = sq->getActiveScene();

    lv_obj_clean(s_bounceBox);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_remove_all_objs(s_bounceGroup);
#endif
    s_bounceText = lv_label_create(s_bounceBox);
    lv_obj_set_style_text_color(s_bounceText, kText, 0);
    lv_obj_set_width(s_bounceText, LV_PCT(100));
    lv_label_set_long_mode(s_bounceText, LV_LABEL_LONG_WRAP);
    set_text_fmt(s_bounceText, "Bouncing Seq %u to track %u...",
                          (unsigned)(scene + 1), (unsigned)(track + 1));
    s_bounceBar = lv_bar_create(s_bounceBox);
    lv_obj_set_size(s_bounceBar, LV_PCT(100), 10);
    lv_bar_set_range(s_bounceBar, 0, 1000);
    lv_obj_set_style_bg_color(s_bounceBar, kPanel, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bounceBar, kRec, LV_PART_INDICATOR);
    s_bounceBtn = bounce_button(s_bounceBox, "Cancel", bounce_close_cb, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_focus_obj(s_bounceBtn);
#endif

    /* The take needs a ring in PSRAM, and PSRAM is what the UI runs out of.
     * Decoded icons are the part of it that costs nothing to give back: they
     * are read again the next time they are drawn. */
    lv_image_cache_drop(nullptr);
    s_bouncing = se->bounceScene(scene, track);
    if (!s_bouncing) {
        lv_label_set_text(s_bounceText, "Could not start: a bounce or the song is already running.");
        lv_label_set_text(lv_obj_get_child(s_bounceBtn, 0), "OK");
    }
}

void bounce_progress()
{
    auto* se = songs();
    if (!se || !s_bounceText) return;
    const crosspad::BounceStatus st = se->bounceStatus();
    if (st.framesTotal) {
        lv_bar_set_value(s_bounceBar, (int32_t)((uint64_t)st.framesDone * 1000 / st.framesTotal), LV_ANIM_OFF);
    }
    if (st.state == crosspad::BounceState::Running) return;
    s_bouncing = false;
    if (st.state == crosspad::BounceState::Done) {
        lv_bar_set_value(s_bounceBar, 1000, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_bounceBar, kPlay, LV_PART_INDICATOR);
        set_text_fmt(s_bounceText, "Done: %s is on its track in Arrange.%s%s", st.file.c_str(),
                              st.error.empty() ? "" : "\nNote: ", st.error.c_str());
    } else {
        set_text_fmt(s_bounceText, "Bounce failed: %s", st.error.c_str());
    }
    lv_label_set_text(lv_obj_get_child(s_bounceBtn, 0), "OK");
}

// ── SRC: the kit, or an instrument borrowed from Pitch Play ────────────────

void src_sync()
{
    if (!s_srcLabel) return;
    if (s_src == Src::Kit) { set_text(s_srcLabel, "KIT"); return; }
    if (!s_instReady.load()) { set_text(s_srcLabel, "..."); return; }
    /* Eight characters of font 10 fit the 48 px button; a longer name is cut
     * with a dot, so a cut reads as one rather than as a misspelling. */
    char buf[16];
    if (s_instName.size() <= 8) std::snprintf(buf, sizeof(buf), "%s", s_instName.c_str());
    else                        std::snprintf(buf, sizeof(buf), "%.7s.", s_instName.c_str());
    set_text(s_srcLabel, buf);
}

/// The pads stop playing the kit and start playing notes. Idempotent.
void src_take_pads()
{
    if (s_padLogicPushed) return;
    auto& pm = crosspad::getPadManager();
    if (!s_srcPadLogic) {
        s_srcPadLogic = std::make_shared<SeqPitchPadLogic>();
        pm.registerPadLogic(kSrcPadLogic, s_srcPadLogic);
    }
    pm.pushPadLogic(kSrcPadLogic);
    s_padLogicPushed = true;
}

/// And back. Pop before anything unregisters the handler, or the pop lands on
/// a name that no longer resolves and leaves the pads with no logic at all.
void src_drop_pads()
{
    if (!s_padLogicPushed) return;
    crosspad::getPadManager().popPadLogic();
    s_padLogicPushed = false;
}

void src_fill();

/// Give the engine back and let the kit load again.
void src_to_kit()
{
    src_drop_pads();
    s_instReady.store(false);
    s_src = Src::Kit;
    s_instPath.clear();
    s_instName.clear();
    s_srcNote.clear();
    if (auto* h = inst_host()) h->release();
    src_sync();
    src_fill();
}

/// Borrow the engine for @p path. The load runs on the host's worker; the
/// pads stay quiet until src_poll() sees Ready.
void src_to_inst(const std::string& path, const std::string& name)
{
    auto* h = inst_host();
    if (!h) { s_srcNote = "No instrument engine on this build."; src_fill(); return; }
    if (!h->requestOpen(path)) { s_srcNote = "Busy -- try again."; src_fill(); return; }
    s_src = Src::Inst;
    s_instPath = path;
    s_instName = name;
    s_instReady.store(false);
    s_srcNote = "Loading " + name + "...";
    src_take_pads();          // before Ready: a hit now is silent, not a kit pad
    src_sync();
    src_fill();
}

/// Host and listing, from the app tick.
void src_poll()
{
    if (s_srcListing) {
        auto* st = tools();
        if (st) {
            const crosspad::ToolJob job = st->job();
            if (job == crosspad::ToolJob::Ready || job == crosspad::ToolJob::Failed) {
                s_srcLib.clear();
                if (job == crosspad::ToolJob::Ready) {
                    const auto f = st->folder();
                    for (size_t i = 0; i < f.folders; ++i) s_srcLib.push_back(f.name(i));
                }
                s_srcListing = false;
                src_fill();
            }
        } else {
            s_srcListing = false;
        }
    }

    if (s_src != Src::Inst) return;
    auto* h = inst_host();
    if (!h) return;
    const auto state = h->state();
    if (state == crosspad::IInstrumentHost::State::Ready) {
        if (!s_instReady.exchange(true)) {
            const std::string msg = h->message();
            s_srcNote = msg.empty() ? std::string() : msg;
            src_sync();
            src_fill();
        }
    } else if (state == crosspad::IInstrumentHost::State::Failed) {
        const std::string why = h->message();
        src_to_kit();
        s_srcNote = why.empty() ? std::string("Could not load that instrument.") : why;
        src_fill();
    }
}

void src_close()
{
    if (!s_srcBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_srcGroup);   // the click that closed it is still being read
    s_srcGroup = nullptr;
#endif
    lv_obj_delete_async(s_srcBox);
    s_srcBox = s_srcList = s_srcNoteLbl = s_srcOctLbl = s_srcChordLbl = nullptr;
    s_srcMinus = s_srcPlus = s_srcClose = s_srcChordMinus = s_srcChordPlus = nullptr;
    sync_ui();
}

void src_close_cb(lv_event_t*) { src_close(); }

void src_pick_cb(lv_event_t* e)
{
    const auto idx = (size_t)(uintptr_t)lv_event_get_user_data(e);
    if (idx == 0) { src_to_kit(); return; }
    const size_t i = idx - 1;
    if (i >= s_srcLib.size()) return;
    auto* h = inst_host();
    if (!h) return;
    const std::string name = s_srcLib[i];
    src_to_inst(h->libraryRoot() + "/" + name + "/" + crosspad::IInstrumentHost::kFileName, name);
}

/// Steps the chord through MusicTheory's list: one note, the types, then the
/// progressions. The map follows at once, so the next hit is the new chord.
void src_chord_cb(lv_event_t* e)
{
    const int d = (int)(intptr_t)lv_event_get_user_data(e);
    const int n = (int)s_mt.totalChordCount();
    s_chordIdx = (s_chordIdx + n + d) % n;
    seq_rebuild_pads();
    if (auto* p = inst()) p->allNotesOff();
    src_fill();
}

void src_oct_cb(lv_event_t* e)
{
    const int d = (int)(intptr_t)lv_event_get_user_data(e);
    const int base = (int)s_base + d * 12;
    /* Sixteen pads from the base, so the top one must still be a note. */
    if (base < 0 || base + (kSeqPads - 1) > 127) return;
    s_base = (uint8_t)base;
    seq_rebuild_pads();
    if (auto* p = inst()) p->allNotesOff();
    src_fill();
}

lv_obj_t* src_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud, bool on)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, on ? kAccent : kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, on ? kInk : kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) lv_group_add_obj(s_srcGroup, b);
#endif
    return b;
}

/// Rebuild the window's contents. Safe when it is closed -- the source can
/// change from the poll, and then there is only the SRC label to update.
void src_fill()
{
    src_sync();
    if (!s_srcList) return;
    /* Only the list is rebuilt. Deleting its buttons takes them out of the
     * group by itself, so nothing else has to be removed -- an earlier
     * lv_group_remove_all_objs() here took OCT and Close out with them and
     * the encoder could never reach them again. */
    lv_obj_clean(s_srcList);

    src_button(s_srcList, "KIT -- the loaded kit", src_pick_cb, (void*)(uintptr_t)0,
               s_src == Src::Kit);
    for (size_t i = 0; i < s_srcLib.size(); ++i) {
        char buf[40];
        std::snprintf(buf, sizeof(buf), LV_SYMBOL_KEYBOARD " %.32s", s_srcLib[i].c_str());
        src_button(s_srcList, buf, src_pick_cb, (void*)(uintptr_t)(i + 1),
                   s_src == Src::Inst && s_instName == s_srcLib[i]);
    }
    if (s_srcLib.empty() && !s_srcListing) {
        lv_obj_t* l = lv_label_create(s_srcList);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(l, kDim, 0);
        lv_obj_set_width(l, LV_PCT(100));
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_label_set_text(l, "No instruments yet -- make one in Pitch Play and SAVE it.");
    }

    if (s_srcOctLbl) {
        char n0[8], n1[8], buf[32];
        note_name(s_ps.padMidiMap[0], n0, sizeof(n0));
        note_name(s_ps.padMidiMap[kSeqPads - 1], n1, sizeof(n1));
        std::snprintf(buf, sizeof(buf), "%s - %s", n0, n1);
        set_text(s_srcOctLbl, buf);
    }
    if (s_srcChordLbl) set_text(s_srcChordLbl, s_mt.chordName((uint8_t)s_chordIdx));
    if (s_srcNoteLbl) set_text(s_srcNoteLbl, s_srcNote.c_str());

#ifdef SEQ_USE_ENCODER
    /* The list's new buttons were appended to the group; move the three
     * below it back behind them so turning the encoder still walks the
     * window top to bottom. */
    if (s_srcGroup) {
        for (lv_obj_t* o : {s_srcMinus, s_srcPlus, s_srcChordMinus, s_srcChordPlus, s_srcClose}) {
            if (!o) continue;
            lv_group_remove_obj(o);
            lv_group_add_obj(s_srcGroup, o);
        }
    }
#endif
}

void src_cb(lv_event_t*)
{
    if (s_srcBox) { src_close(); return; }

    s_srcBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_srcBox);
    lv_obj_set_size(s_srcBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_srcBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_srcBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_srcBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_srcBox, 8, 0);
    lv_obj_set_style_pad_row(s_srcBox, 4, 0);
    lv_obj_set_flex_flow(s_srcBox, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_srcBox, LV_OBJ_FLAG_SCROLLABLE);

#ifdef SEQ_USE_ENCODER
    s_srcGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_srcGroup);
#endif

    lv_obj_t* title = lv_label_create(s_srcBox);
    lv_obj_set_style_text_color(title, kText, 0);
    lv_label_set_text(title, "What the pads record");

    s_srcList = lv_obj_create(s_srcBox);
    lv_obj_remove_style_all(s_srcList);
    lv_obj_set_width(s_srcList, LV_PCT(100));
    lv_obj_set_flex_grow(s_srcList, 1);
    lv_obj_set_flex_flow(s_srcList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_srcList, 3, 0);

    /* The sixteen pads' range, for an instrument. Shown always, so the window
     * does not change shape when the source does. */
    lv_obj_t* oct = make_row(s_srcBox, 24);
    lv_obj_t* ol = lv_label_create(oct);
    lv_obj_set_style_text_font(ol, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(ol, kDim, 0);
    lv_label_set_text(ol, "Pads");
    lv_obj_t* minus = lv_button_create(oct);
    lv_obj_set_size(minus, 28, 22);
    lv_obj_set_style_radius(minus, 6, 0);
    lv_obj_set_style_shadow_width(minus, 0, 0);
    lv_obj_set_style_bg_color(minus, kPanel, 0);
    s_srcMinus = minus;
    lv_obj_add_event_cb(minus, src_oct_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_center(lv_label_create(minus));
    lv_label_set_text(lv_obj_get_child(minus, 0), LV_SYMBOL_MINUS);
    s_srcOctLbl = lv_label_create(oct);
    lv_obj_set_width(s_srcOctLbl, 70);
    lv_obj_set_style_text_align(s_srcOctLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_srcOctLbl, kText, 0);
    lv_label_set_text(s_srcOctLbl, "");
    lv_obj_t* plus = lv_button_create(oct);
    lv_obj_set_size(plus, 28, 22);
    lv_obj_set_style_radius(plus, 6, 0);
    lv_obj_set_style_shadow_width(plus, 0, 0);
    lv_obj_set_style_bg_color(plus, kPanel, 0);
    s_srcPlus = plus;
    lv_obj_add_event_cb(plus, src_oct_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_center(lv_label_create(plus));
    lv_label_set_text(lv_obj_get_child(plus, 0), LV_SYMBOL_PLUS);
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_add_obj(s_srcGroup, minus); lv_group_add_obj(s_srcGroup, plus); }
#endif

    /* What a pad makes: one note, a chord type, or a progression -- the same
     * list Pitch Play offers, because it is the same engine underneath. */
    lv_obj_t* chord = make_row(s_srcBox, 24);
    lv_obj_t* cl = lv_label_create(chord);
    lv_obj_set_style_text_font(cl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(cl, kDim, 0);
    lv_label_set_text(cl, "Chord");
    lv_obj_t* cminus = lv_button_create(chord);
    lv_obj_set_size(cminus, 28, 22);
    lv_obj_set_style_radius(cminus, 6, 0);
    lv_obj_set_style_shadow_width(cminus, 0, 0);
    lv_obj_set_style_bg_color(cminus, kPanel, 0);
    s_srcChordMinus = cminus;
    lv_obj_add_event_cb(cminus, src_chord_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_center(lv_label_create(cminus));
    lv_label_set_text(lv_obj_get_child(cminus, 0), LV_SYMBOL_MINUS);
    s_srcChordLbl = lv_label_create(chord);
    lv_obj_set_flex_grow(s_srcChordLbl, 1);
    lv_obj_set_style_text_align(s_srcChordLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_srcChordLbl, kAccent, 0);
    lv_label_set_text(s_srcChordLbl, "");
    lv_obj_t* cplus = lv_button_create(chord);
    lv_obj_set_size(cplus, 28, 22);
    lv_obj_set_style_radius(cplus, 6, 0);
    lv_obj_set_style_shadow_width(cplus, 0, 0);
    lv_obj_set_style_bg_color(cplus, kPanel, 0);
    s_srcChordPlus = cplus;
    lv_obj_add_event_cb(cplus, src_chord_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_center(lv_label_create(cplus));
    lv_label_set_text(lv_obj_get_child(cplus, 0), LV_SYMBOL_PLUS);
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_add_obj(s_srcGroup, cminus); lv_group_add_obj(s_srcGroup, cplus); }
#endif

    s_srcNoteLbl = lv_label_create(s_srcBox);
    lv_obj_set_style_text_font(s_srcNoteLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_srcNoteLbl, kAmber, 0);
    lv_obj_set_width(s_srcNoteLbl, LV_PCT(100));
    lv_label_set_long_mode(s_srcNoteLbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_srcNoteLbl, "");

    lv_obj_t* close = lv_button_create(s_srcBox);
    lv_obj_set_size(close, LV_PCT(100), 24);
    lv_obj_set_style_radius(close, 6, 0);
    lv_obj_set_style_shadow_width(close, 0, 0);
    lv_obj_set_style_bg_color(close, kPanel, 0);
    s_srcClose = close;
    lv_obj_add_event_cb(close, src_close_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_center(lv_label_create(close));
    lv_label_set_text(lv_obj_get_child(close, 0), "Close");
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_add_obj(s_srcGroup, close); lv_group_focus_obj(close); }
#endif

    /* Ask the card what is in the library; src_poll() fills the list when the
     * answer comes. The window opens with whatever was last read. */
    if (auto* st = tools()) {
        if (auto* h = inst_host()) {
            if (st->requestFolder(st->cardRoot() + h->libraryRoot())) s_srcListing = true;
        }
    }
    src_fill();
}

// ── swing ──────────────────────────────────────────────────────────────────

/* Swing is late, not moved: every second subdivision sounds a share of one
 * later, the pattern keeps its straight positions, and turning it off puts the
 * feel back exactly where it was. It is applied on the way out (see
 * PatternSequencer::swung), and a take recorded while it is on is pulled back
 * to the straight grid first -- the player aims at what they hear.
 *
 * Its own window because it is two settings, not one: how much, and which
 * subdivision. A button that only cycled through amounts would make the second
 * one unreachable, and a hold-to-edit knob would hide both. */

void swing_close()
{
    if (!s_swingBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_swingGroup);   // the click that closed it is still being read
    s_swingGroup = nullptr;
#endif
    lv_obj_delete_async(s_swingBox);
    s_swingBox = s_swingText = s_swingVal = nullptr;
    for (auto& c : s_swingChip) c = nullptr;
    sync_ui();
}

void swing_close_cb(lv_event_t*) { swing_close(); }

/* The steps a sampler's swing knob clicks through, 50 % to 75 %, and what each
 * is on the 0..100 scale: intensity = (sampler - 50) * 4. The triplet one is
 * the exception -- those machines print 66 % and mean two thirds of the way
 * through the pair, which is 67 here, so the chip says 66 and sets the feel
 * rather than the rounding. */
struct SwingStep { uint8_t sampler; uint8_t intensity; };
const SwingStep kSwingSteps[7] = {
    {50, 0}, {54, 16}, {58, 32}, {62, 48}, {66, 67}, {70, 80}, {75, 100},
};

/// What it is set to, in the two numbers a player might be thinking in.
void swing_paint()
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t pct = sq->getSwing();
    const uint8_t per = sq->getSwingPerBeat();
    if (s_swingVal) {
        char v[8];
        std::snprintf(v, sizeof(v), "%u", (unsigned)pct);
        set_text(lv_obj_get_child(s_swingVal, 0), v);
    }
    if (s_swingText) {
        if (!pct) {
            set_text(s_swingText, "straight -- every subdivision on its own line");
        } else {
            set_text_fmt(s_swingText, "1/%u   .   sampler %u %%%s",
                         (unsigned)(per * 4), (unsigned)(50 + pct / 4),
                         pct >= 66 && pct <= 68 ? "   .   triplet feel" : "");
        }
    }
    /* The step it is standing on, lit. Turned off a step, nothing is lit --
     * which is the honest answer. */
    for (int i = 0; i < 7; ++i) {
        if (!s_swingChip[i]) continue;
        const bool on = kSwingSteps[i].intensity == pct;
        lv_obj_set_style_bg_color(s_swingChip[i], on ? kAmber : kPanel, 0);
        if (lv_obj_t* l = lv_obj_get_child(s_swingChip[i], 0)) {
            lv_obj_set_style_text_color(l, on ? kInk : kText, 0);
        }
    }
}

/// The amount, turned: focus it, press to take it, a point per detent, press
/// again to let go -- the same way the tempo is turned on the screen behind.
void swing_val_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    if (!sq || !b) return;
    lv_group_t* g = lv_obj_get_group(b);
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int step = key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1
                   : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
    if (!step) return;
    const int now = (int)sq->getSwing() + step;
    sq->setSwing((uint8_t)(now < 0 ? 0 : (now > 100 ? 100 : now)));
    swing_paint();
}

void swing_set_cb(lv_event_t* e)
{
    auto* sq = seq();
    if (!sq) return;
    sq->setSwing((uint8_t)(intptr_t)lv_event_get_user_data(e));
    swing_paint();
}

void swing_unit_cb(lv_event_t* e)
{
    auto* sq = seq();
    if (!sq) return;
    sq->setSwingPerBeat(sq->getSwingPerBeat() == 2 ? 4 : 2);
    swing_paint();
    /* The button says which one it is, so it has to be repainted too. */
    if (lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e))) {
        if (lv_obj_t* l = lv_obj_get_child(b, 0)) {
            set_text_fmt(l, "Swing the 1/%u", (unsigned)(sq->getSwingPerBeat() * 4));
        }
    }
}

void swing_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq || s_swingBox) return;

    s_swingBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_swingBox);
    lv_obj_set_size(s_swingBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_swingBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_swingBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_swingBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_swingBox, 8, 0);
    lv_obj_set_style_pad_row(s_swingBox, 4, 0);
    lv_obj_set_flex_flow(s_swingBox, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_swingBox, LV_OBJ_FLAG_SCROLLABLE);
#ifdef SEQ_USE_ENCODER
    s_swingGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_swingGroup);
#endif

    lv_obj_t* title = lv_label_create(s_swingBox);
    lv_obj_set_style_text_color(title, kText, 0);
    lv_label_set_text(title, "SWING");

    s_swingText = lv_label_create(s_swingBox);
    lv_obj_set_style_text_font(s_swingText, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_swingText, kAmber, 0);
    lv_label_set_long_mode(s_swingText, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_swingText, LV_PCT(100));
    lv_label_set_text(s_swingText, "");

    /* The amount, big, with the scale beside it: one knob from nothing to as
     * far as those machines go. */
    lv_obj_t* amount = make_row(s_swingBox, 30);
    lv_obj_t* tag = lv_label_create(amount);
    lv_obj_set_style_text_font(tag, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(tag, kDim, 0);
    lv_label_set_text(tag, "AMOUNT");
    s_swingVal = swing_value(amount);
    lv_obj_t* of = lv_label_create(amount);
    lv_obj_set_style_text_font(of, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(of, kDim, 0);
    lv_label_set_text(of, "of 100");

    char unit[20];
    lv_snprintf(unit, sizeof(unit), "Swing the 1/%u", (unsigned)(sq->getSwingPerBeat() * 4));
    swing_button(s_swingBox, unit, swing_unit_cb, nullptr);

    /* The sampler's own steps, in a row: the numbers printed on those machines,
     * so a groove learned on one lands here by name. */
    lv_obj_t* steps = make_row(s_swingBox, 26);
    for (int i = 0; i < 7; ++i) {
        char label[8];
        lv_snprintf(label, sizeof(label), "%u", (unsigned)kSwingSteps[i].sampler);
        s_swingChip[i] = swing_chip(steps, label, (void*)(intptr_t)kSwingSteps[i].intensity);
    }

    swing_button(s_swingBox, LV_SYMBOL_OK "  Done", swing_close_cb, nullptr);

    swing_paint();
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_focus_obj(s_swingVal);
#endif
}

void bounce_cb(lv_event_t*)
{
    auto* se = songs();
    auto* sq = seq();
    if (!se || !sq || s_bounceBox) return;

    s_bounceBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_bounceBox);
    lv_obj_set_size(s_bounceBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_bounceBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_bounceBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_bounceBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_bounceBox, 8, 0);
    lv_obj_set_style_pad_row(s_bounceBox, 5, 0);
    lv_obj_set_flex_flow(s_bounceBox, LV_FLEX_FLOW_COLUMN);
#ifdef SEQ_USE_ENCODER
    s_bounceGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_bounceGroup);
#endif

    std::string project;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        project = se->song().name;
    }
    lv_obj_t* t = lv_label_create(s_bounceBox);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_obj_set_width(t, LV_PCT(100));
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    set_text_fmt(t, "Bounce Seq %u (%u bars) into \"%s\" -- on which track?",
                          (unsigned)(sq->getActiveScene() + 1),
                          (unsigned)sq->getSceneBars(sq->getActiveScene()),
                          project.empty() ? "no card" : project.c_str());
    for (int i = 0; i < crosspad::AudioSong::kTracks; ++i) {
        char label[16];
        lv_snprintf(label, sizeof(label), "Track %d", i + 1);
        bounce_button(s_bounceBox, label, bounce_track_cb, (void*)(intptr_t)i);
    }
    bounce_button(s_bounceBox, "Cancel", bounce_close_cb, nullptr);
}

// ── building ───────────────────────────────────────────────────────────────

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, int32_t h, const char* text,
                      lv_event_cb_t cb, void* ud = nullptr)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

lv_obj_t* make_row(lv_obj_t* parent, int32_t h)
{
    lv_obj_t* r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), h);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, 3, 0);
    lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

/// Tempo: focus it, press to edit, turn a BPM at a time, press again to
/// leave. A button that shows the whole number, not a spinbox: the spinbox is
/// a text area that scrolls to its cursor, and in edit mode it showed only the
/// last digit. A tap puts it in edit mode too, for the encoder to turn.
lv_obj_t* make_bpm(lv_obj_t* parent)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    lv_obj_set_size(b, 44, 22);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    /* On the button: only it is ever EDITED; the label takes the colour. */
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_CLICKED, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

} // namespace

lv_obj_t* Sequencer_create(lv_obj_t* parent, App* app)
{
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 5, 0);
    lv_obj_set_style_pad_row(s_root, 4, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    // -- what the scene is counted in --
    lv_obj_t* top = make_row(s_root, 22);
    make_button(top, 30, 22, LV_SYMBOL_LEFT, back_cb, app);
    s_bpm = make_bpm(top);
    if (!s_tempoSub) {
        /* Event-bus task: only remember who drives the tempo; sync_ui() on
         * the LVGL timer shows it. */
        s_tempoSub = crosspad::getEventBus().subscribe(crosspad::EventType::TempoChanged,
            [](const void* data, crosspad::EventType, void*) {
                const auto* d = static_cast<const crosspad::TempoChangedData*>(data);
                s_tempoFromHost.store(d->source == crosspad::EventSource::MidiInput,
                                      std::memory_order_relaxed);
            }, nullptr);
    }
    s_clickBtn = make_button(top, 26, 22, "M", click_cb);
    s_preBtn = make_button(top, 30, 22, "PRE", pre_cb);
    lv_obj_t* barsBtn = make_button(top, 26, 22, "4b", bars_cb);
    s_barsLabel = lv_obj_get_child(barsBtn, 0);
    s_quantBtn = make_button(top, 36, 22, "1/16", quant_cb);
    s_quantLabel = lv_obj_get_child(s_quantBtn, 0);
    /* Beside the quantisation, because they are the same kind of thing: where
     * a hit is put, and where it is heard. x2 and /2 give up six pixels each
     * for it -- their labels are two characters. */
    s_swingBtn = make_button(top, 44, 22, "SWING", swing_cb);
    make_button(top, 24, 22, "x2", double_cb);
    make_button(top, 24, 22, "/2", halve_cb);

    // -- which scene, and ERASE held beside it --
    lv_obj_t* scenes = make_row(s_root, 22);
    make_button(scenes, 26, 22, LV_SYMBOL_LEFT, scene_prev_cb);
    s_sceneLabel = lv_label_create(scenes);
    lv_obj_set_width(s_sceneLabel, 64);
    lv_obj_set_style_text_align(s_sceneLabel, LV_TEXT_ALIGN_CENTER, 0);
    make_button(scenes, 26, 22, LV_SYMBOL_RIGHT, scene_next_cb);
    make_button(scenes, 26, 22, LV_SYMBOL_MINUS, scene_del_cb);
    make_button(scenes, 26, 22, LV_SYMBOL_PLUS, scene_add_cb);
    lv_obj_t* gap = lv_obj_create(scenes);
    lv_obj_remove_style_all(gap);
    lv_obj_set_flex_grow(gap, 1);
    make_button(scenes, 50, 22, "BOUNCE", bounce_cb);
    s_eraseBtn = make_button(scenes, 60, 22, "ERASE", nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESS_LOST, nullptr);

    // -- the pattern --
    s_grid = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_grid);
    lv_obj_set_width(s_grid, LV_PCT(100));
    lv_obj_set_flex_grow(s_grid, 1);
    lv_obj_set_style_bg_color(s_grid, kPanel, 0);
    lv_obj_set_style_bg_opa(s_grid, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_grid, 6, 0);
    lv_obj_set_style_pad_all(s_grid, 3, 0);
    lv_obj_remove_flag(s_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_grid, grid_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

    // -- transport --
    /* Six at 48 rather than five at 58: 6*48 + 5*3 = 303 of the 310 a padded
     * root leaves. SRC sits beside REC because it is the other half of the
     * same question -- what a take is made of. */
    lv_obj_t* tr = make_row(s_root, 28);
    s_srcBtn  = make_button(tr, 48, 28, "KIT", src_cb);
    s_srcLabel = lv_obj_get_child(s_srcBtn, 0);
    s_recBtn  = make_button(tr, 48, 28, "REC", rec_cb);
    s_dubBtn  = make_button(tr, 48, 28, "DUB", dub_cb);
    s_playBtn = make_button(tr, 48, 28, LV_SYMBOL_PLAY, play_cb);
    s_stopBtn = make_button(tr, 48, 28, LV_SYMBOL_STOP, stop_cb);
    s_syncBtn = make_button(tr, 48, 28, "SYNC", sync_cb);
    src_sync();

    seq_rebuild_pads();
    sync_ui();
    s_drawn = pattern_shape();
    s_headX = -1;

#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_focus_obj(s_playBtn);
#endif

    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef SEQ_USE_ENCODER
            if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
            /* Erase must never outlive the button holding it. */
            if (auto* sq = seq()) sq->setErasing(false);
#ifdef SEQ_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
#endif
        });
    }

    return s_root;
}

void Sequencer_destroy(lv_obj_t* root)
{
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    /* A screen closed with erase still held must not leave every later hit
     * erasing instead of playing. The loop itself keeps running, and so does
     * a bounce: it finishes on its own and lands in the song. */
    if (auto* sq = seq()) sq->setErasing(false);

    /* The pads go back to the kit's. The engine too -- unless a bounce is
     * still capturing them, which it does on its own after the screen is
     * gone: reloading the kit under it would record the wrong instrument.
     * It is then held until SRC is set back to KIT, or Pitch Play takes it. */
    src_drop_pads();
    if (s_src == Src::Inst && !s_bouncing) {
        if (auto* h = inst_host()) h->release();
        s_src = Src::Kit;
        s_instPath.clear();
        s_instName.clear();
    }
    s_instReady.store(false);
    s_srcNote.clear();
    s_srcListing = false;

#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_delete(s_srcGroup); s_srcGroup = nullptr; }
    if (s_bounceGroup) { lv_group_delete(s_bounceGroup); s_bounceGroup = nullptr; }
    if (s_swingGroup) { lv_group_delete(s_swingGroup); s_swingGroup = nullptr; }
#endif
    s_bounceBox = s_bounceText = s_bounceBar = s_bounceBtn = nullptr;
    s_swingBox = s_swingText = s_swingVal = s_swingBtn = nullptr;
    s_srcBox = s_srcList = s_srcNoteLbl = s_srcOctLbl = s_srcBtn = s_srcLabel = nullptr;
    s_srcChordLbl = nullptr;
    s_srcMinus = s_srcPlus = s_srcClose = s_srcChordMinus = s_srcChordPlus = nullptr;
    for (auto& c : s_swingChip) c = nullptr;
    s_bouncing = false;
    s_root = s_grid = s_bpm = s_syncBtn = s_clickBtn = s_preBtn = s_barsLabel = nullptr;
    s_quantBtn = s_quantLabel = s_sceneLabel = s_eraseBtn = nullptr;
    s_recBtn = s_dubBtn = s_playBtn = s_stopBtn = nullptr;

#ifdef SEQ_USE_ENCODER
    // Hand navigation back before the group dies, or the launcher comes back
    // unnavigable and only a reboot fixes it.
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif

    lv_obj_delete_async(root);
}

// Auto-registration: the app shows up in the launcher with no wiring anywhere
// else. No pad logic: the pads stay the Sampler's, so they play and record here
// exactly as they do everywhere else.
REGISTER_APP(Sequencer, nullptr, "sequencer.png",
             Sequencer_create, Sequencer_destroy,
             nullptr, nullptr, nullptr, 0)
