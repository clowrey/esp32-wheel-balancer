#pragma once
// Touchscreen UI (PLAN §7), 1024x600 landscape, drawn straight into an RGB565 frame (no LVGL).
// Talks only to UiModel, so the same code runs on the device, in the PC simulator
// (tools/ui_sim) and in the host tests. How a screen is written: ui_kit.h.
//
// Each frame() runs the open screen twice without drawing (a "dry run" that sums up what each
// 16-row band of the frame would show), then draws only the bands that would look different
// from what is on the panel. The caller sends those rows to the display.

#include <functional>
#include <string>

#include "ui_canvas.h"
#include "ui_kit.h"
#include "ui_model.h"

namespace bal {

class BalancerUi {
 public:
  enum Screen { HOME = 0, SETUP, MEASURE, RESULT, LIVE, SETTINGS, SCREEN_COUNT };

  void bind(UiModel *model) { model_ = model; }
  bool built() const { return model_ != nullptr; }

  // One frame. Give it the touch panel as it is now; the screen reacts to it and calls the
  // model. Draws again only the bands of rows that would look different from the last frame
  // (listed in `redrawn`) and returns whether there were any. Cheap when nothing changed.
  bool frame(Canvas &c, const Input &input, uint32_t now_ms, HitTrace *trace = nullptr);
  // The next frame() draws everything (the canvas was used for something else)
  void redraw() { drawn_ = false; }

  void show(Screen s);
  Screen current() const { return current_; }

  // The rows the last frame() drew again
  struct Rows {
    int y0, y1;  // rows y0 .. y1 - 1
  };
  static const int MAX_RUNS = 8;
  Rows redrawn[MAX_RUNS];
  int redrawn_count = 0;

  // For the simulator and tests
  void press_start() { on_start_(); }
  void press_accept() { on_accept_(); }
  void press_verify() { on_verify_(); }
  const std::string &toast_text() const { return toast_; }
  bool number_pad_open() const { return pad_.open; }
  bool axis_finder_open() const { return det_ch_ >= 0; }
  void set_show_raw(bool on) { l_show_raw_ = on; }

 private:
  // ---- the frame ----
  void tick_(uint32_t now);  // what moves with time: the axis finder, the toast, the speed lock
  void draw_(Frame &f);
  void bar_(Frame &f, const char *title);
  void draw_toast_(Frame &f);

  // ---- screens (ui_screens.cpp) ----
  void home_(Frame &f);
  void setup_(Frame &f);
  void measure_(Frame &f);
  void result_(Frame &f);
  void live_(Frame &f);
  void settings_(Frame &f);
  void detect_(Frame &f);  // the guided axis finder, over the settings screen
  void open_detect_();
  void detect_next_();
  void finish_detect_();

  // ---- pop-ups ----
  void ask_number_(const char *title, double current, int decimals, std::function<void(double)> on_ok);
  void number_pad_(Frame &f);
  void toast_msg_(const std::string &msg) { toast_ = msg, toast_until_ = now_ms_ + 3500; }

  // ---- actions ----
  void on_start_();
  void on_accept_();
  void on_verify_();

  std::string mass_(double g) const;         // in g or oz, as set on the settings screen
  std::string mass_pieces_(double g) const;  // nearest 1 oz + 1/4 oz stack (oz mode only)
  // A row whose value is a number: shows it with its unit and opens the pad when tapped
  bool number_row_(Frame &f, int x, int y, int w, const char *name, const char *unit, int decimals, double &value,
                   double lo, double hi);

  UiModel *model_ = nullptr;
  Screen current_ = HOME;
  uint32_t now_ms_ = 0;

  // touch, between frames
  bool was_down_ = false, dragging_ = false;
  int last_x_ = 0, last_y_ = 0, press_x_ = 0, press_y_ = 0;
  // what the panel shows, summed up band by band
  bool drawn_ = false;
  uint32_t drawn_sums_[Canvas::MAX_BANDS] = {};

  // pop-ups
  struct NumberPad {
    bool open = false;
    bool fresh = true;  // the first key replaces the value shown
    std::string title;
    char entry[24] = "";
    std::function<void(double)> ok;
  } pad_;
  std::string toast_;
  uint32_t toast_until_ = 0;

  // setup
  int plane_sel_ = 0;
  // measure
  double m_prev_test_rpm_ = 0;  // to notice the speed locking by itself
  // live
  bool l_show_raw_ = false;
  // guided axis finder
  int det_ch_ = -1;  // sensor being found, -1 closed
  bool det_running_ = false, det_done_ = false;
  uint32_t det_end_ms_ = 0;
  float det_peak_[3] = {0, 0, 0};
  std::string det_result_;
};

}  // namespace bal
