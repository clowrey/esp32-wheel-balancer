#pragma once
// Touchscreen UI (PLAN §7) in LVGL 9, 1024x600 landscape. Talks only to
// UiModel, so the same code runs on the device and in tools/ui_preview.

#include <functional>
#include <string>
#include <vector>

#include "lvgl.h"
#include "ui_model.h"

namespace bal {

// Rotor dial: tach mark at the top, angles increasing clockwise on screen,
// rotation arrow, fixed positions, and the correction weight marker.
class DialView {
 public:
  void create(lv_obj_t *parent, int x, int y, int size);
  // angle in user convention; rotation_ccw: which way the rotor turns on screen
  void set(bool visible, double angle_deg, bool remove, int positions, double positions_offset,
           int split_a, int split_b, bool rotation_ccw);
  lv_obj_t *root() const { return root_; }

 private:
  lv_obj_t *root_ = nullptr;
  int size_ = 0;
  lv_obj_t *marker_ = nullptr;
  lv_obj_t *marker_line_ = nullptr;
  lv_point_precise_t marker_pts_[2];
  lv_obj_t *arrow_ = nullptr;
  lv_point_precise_t arrow_pts_[12];
  std::vector<lv_obj_t *> spots_;
  std::vector<lv_obj_t *> spot_labels_;
  lv_point_precise_t tick_pts_[12][2];
};

// Polar plot of the 1x vector: recent revolutions as dots, mean as a line.
class PolarView {
 public:
  void create(lv_obj_t *parent, int x, int y, int size, lv_color_t color);
  void set(const std::vector<cplx> &dots, cplx mean, bool have_mean, const char *unit);
  void attach_scale_label(lv_obj_t *l) { scale_label_ = l; }

 private:
  static constexpr int kDots = 40;
  lv_obj_t *root_ = nullptr;
  int size_ = 0;
  lv_obj_t *dots_[kDots];
  lv_obj_t *mean_line_ = nullptr;
  lv_point_precise_t mean_pts_[2];
  lv_obj_t *mean_head_ = nullptr;
  lv_obj_t *scale_label_ = nullptr;
  lv_point_precise_t axis_pts_[2][2];
};

class BalancerUi {
 public:
  enum Screen { HOME = 0, SETUP, MEASURE, RESULT, LIVE, SETTINGS, SCREEN_COUNT };

  void build(UiModel *model);
  bool built() const { return model_ != nullptr; }
  void update();  // call a few times a second
  void show(Screen s);
  Screen current() const { return current_; }

  // Hooks for the preview harness.
  void press_start() { on_start_(); }
  void press_accept() { on_accept_(); }
  void press_verify() { on_verify_(); }

 private:
  struct Bar {
    lv_obj_t *title = nullptr, *rpm = nullptr, *status = nullptr;
  };
  struct Row {
    lv_obj_t *button = nullptr;
    lv_obj_t *value = nullptr;
    std::function<std::string()> get;
  };

  lv_obj_t *make_screen_(Screen s, const char *title);
  lv_obj_t *button_(lv_obj_t *parent, const char *text, const lv_font_t *font, std::function<void()> fn,
                    lv_color_t color);
  Row &row_(std::vector<Row> &rows, lv_obj_t *parent, const char *name, std::function<std::string()> get,
            std::function<void()> on_tap);
  void number_row_(std::vector<Row> &rows, lv_obj_t *parent, const char *name, const char *unit, int decimals,
                   std::function<double &()> ref, double lo, double hi, std::function<void()> after = nullptr);
  void ask_number_(const char *title, double current, int decimals, std::function<void(double)> on_ok);
  void refresh_rows_(std::vector<Row> &rows);
  void toast_(const std::string &msg);

  void build_home_();
  void build_setup_();
  void build_measure_();
  void build_result_();
  void build_live_();
  void build_settings_();

  void update_bar_();
  void update_home_();
  void update_measure_();
  void update_result_();
  void update_live_();
  void update_settings_();
  std::string mass_(double g) const;         // in g or oz, as set on the settings screen
  std::string mass_pieces_(double g) const;  // nearest 1 oz + 1/4 oz stack (oz mode only)
  void build_detect_(lv_obj_t *scr);
  void open_detect_();
  void detect_next_();
  void finish_detect_();
  void update_detect_();
  void rebuild_plane_rows_();

  void on_start_();
  void on_accept_();
  void on_verify_();

  UiModel *model_ = nullptr;
  Screen current_ = HOME;
  lv_obj_t *scr_[SCREEN_COUNT] = {};
  Bar bar_[SCREEN_COUNT];

  // home
  lv_obj_t *home_trim_btn_ = nullptr, *home_info_ = nullptr, *home_demo_ = nullptr;
  // setup
  std::vector<Row> setup_rows_;
  std::vector<Row> plane_rows_;
  lv_obj_t *plane_panel_ = nullptr;
  lv_obj_t *plane_tab_[2] = {};
  int plane_sel_ = 0;
  lv_obj_t *suggest_label_ = nullptr;
  // measure
  lv_obj_t *m_instr_ = nullptr, *m_step_ = nullptr, *m_rpm_ = nullptr, *m_gauge_ = nullptr, *m_band_ = nullptr;
  lv_obj_t *m_window_ = nullptr, *m_state_ = nullptr, *m_revs_ = nullptr, *m_revs_bar_ = nullptr;
  lv_obj_t *m_lock_ = nullptr, *m_setspeed_ = nullptr, *m_spin_ = nullptr, *m_accept_ = nullptr;
  lv_obj_t *m_read_[2] = {};
  PolarView polar_[2];
  // result
  lv_obj_t *r_instr_ = nullptr, *r_big_[2] = {}, *r_detail_[2] = {}, *r_title_[2] = {}, *r_info_ = nullptr;
  lv_obj_t *r_verify_ = nullptr;
  DialView dial_[2];
  // live
  lv_obj_t *l_rpm_ = nullptr, *l_ch_[2] = {}, *l_diag_ = nullptr, *l_spin_ = nullptr;
  lv_obj_t *l_wave_[2] = {};
  lv_point_precise_t l_wave_pts_[2][kWaveBins + 1];
  lv_obj_t *l_bars_[2][kMaxOrder] = {};
  // Bottom-right panel shows either the orders or the raw readings.
  lv_obj_t *s_teach_btn_ = nullptr;  // settings: hold to teach the tach sensor
  // settings: axis picker per sensor, and network status
  lv_obj_t *s_axis_btn_[2][3] = {};
  lv_obj_t *s_axis_info_[2] = {}, *s_axis_grav_[2] = {};
  lv_obj_t *s_net_ = nullptr;
  // guided axis finder (overlay on the settings screen)
  lv_obj_t *det_overlay_ = nullptr, *det_title_ = nullptr, *det_text_ = nullptr, *det_prog_ = nullptr;
  lv_obj_t *det_bar_[3] = {}, *det_val_[3] = {}, *det_btn_l_ = nullptr, *det_btn_r_ = nullptr;
  lv_obj_t *det_line_a_[3] = {}, *det_line_b_[3] = {};
  lv_point_precise_t det_axle_pts_[2], det_arrow_a_[2], det_head_a_[3], det_head_a2_[3];
  lv_point_precise_t det_arrow_b_[2], det_head_b_[3], det_head_b2_[3];
  int det_ch_ = -1;  // sensor being found, -1 closed
  bool det_running_ = false, det_done_ = false;
  uint32_t det_end_ms_ = 0;
  float det_peak_[3] = {0, 0, 0};
  std::string det_result_;
  lv_obj_t *l_orders_box_ = nullptr, *l_raw_box_ = nullptr, *l_view_btn_ = nullptr, *l_pclog_btn_ = nullptr;
  lv_obj_t *l_raw_[2] = {}, *l_raw_info_ = nullptr;
  lv_point_precise_t l_raw_pts_[2][LiveData::kRawPts];
  bool l_show_raw_ = false;
  // settings
  std::vector<Row> settings_rows_;
};

}  // namespace bal
