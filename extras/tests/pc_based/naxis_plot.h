// naxis_plot.h — gnuplot dump helper for the FasNAxis PC tests.
//
// Writes one self-contained multi-panel gnuplot file the way RampChecker does
// (RampChecker::start_plot / finish_plot in test_02): open a heredoc, stream
// rows, close it, then emit the "set multiplot layout" blocks. Panel 1 is the
// XY figure — the commanded polyline in grey with the realized path on top —
// which is what "one figure should show the xy-plot for 2 steppers" means.
// Panels 2..4 overlay one series per axis: speed [steps/s] over time,
// performed-vs-remaining ramp steps P/R over time, and period [ticks] over
// time. Panel 5 plots the step deviation distance (commanded (x,y) minus
// realized (x,y)) over time on its own scale: the 1600x1600 XY path is too
// coarse to reveal small per-step deviations, so they need their own panel.
// Speeds use the test-only TICKS_PER_S / ticks conversion (whitepaper
// section 6.1 / 13.1); the helper holds no kinematics.
//
// Each panel is a single newline-terminated "plot ... series, ... series"
// statement (gnuplot does not accept a literal backslash-n line continuation).
// "make clean" already deletes *.gnuplot.
//
// Protocol:
//   plot.start_plot("f5", "FasNAxis F5 square", 2);
//   for (waypoint w in polyline) plot.poly_point(w.x, w.y);
//   plot.poly_done();                           // close the commanded-path
//   heredoc for (sample s in run)
//     plot.row(s.t, s.x, s.y, s.dev, s.speed, s.P, s.R, s.ticks);
//   plot.finish_plot();
//
#ifndef NAXIS_PLOT_H
#define NAXIS_PLOT_H

#include <stdio.h>
#include <string.h>

// Up to this many axes get per-axis overlay series (2D and 3D fixtures use the
// first N). The XY panel always uses the first two axes.
#define NAXIS_PLOT_MAX 3

// Test-only conversion from ticks to steps/s; never formed in production.
#define NAXIS_PLOT_TICKS_PER_S 16000000.0

class NaxisPlot {
 public:
  int n_axes;

  NaxisPlot() : open(false), title("FasNAxis"), n_axes(1) {}

  void start_plot(const char* fixture, const char* fixture_title, int axes) {
    n_axes = axes;
    if (n_axes > NAXIS_PLOT_MAX) {
      n_axes = NAXIS_PLOT_MAX;
    }
    snprintf(title, sizeof(title), "%s", fixture_title);
    snprintf(filename, sizeof(filename), "test_26_%s.gnuplot", fixture);
    gp = fopen(filename, "w");
    if (gp == NULL) {
      open = false;
      return;
    }
    open = true;
    // Panel 1 references a separate commanded-path heredoc, written up front.
    fprintf(gp, "$poly <<EOF\n");
  }

  // One commanded polyline vertex (grey reference line in panel 1).
  void poly_point(double x, double y) {
    if (open) {
      fprintf(gp, "%.6f %.6f\n", x, y);
    }
  }

  // Closes the commanded-path heredoc and opens the per-sample data heredoc.
  // Column layout of every data row:
  //      1: t [s]      2: x      3: y      4: deviation (steps,
  //      commanded-realized) then per axis i (0-based): 5+4i: speed [steps/s]
  //      6+4i: P   7+4i: R   8+4i: period [ticks]
  void poly_done() {
    if (open) {
      fprintf(gp, "EOF\n");
      fprintf(gp, "$data <<EOF\n");
    }
  }

  void row(double t, double x, double y, double deviation, const double* speed,
           const double* P, const double* R, const double* ticks) {
    if (!open) {
      return;
    }
    fprintf(gp, "%.6f %.6f %.6f %.6f", t, x, y, deviation);
    for (int i = 0; i < n_axes; i++) {
      fprintf(gp, " %.6f %d %d %d", speed[i], (int)P[i], (int)R[i],
              (int)ticks[i]);
    }
    fprintf(gp, "\n");
  }

  void finish_plot() {
    if (!open) {
      return;
    }
    fprintf(gp, "EOF\n");
    fprintf(gp, "set term pngcairo size 1600, 1200\n");
    fprintf(gp, "set output \"%s.png\"\n", filename);
    fprintf(gp, "set multiplot layout 3,2\n");

    // Panel 1: XY — grey commanded polyline with the realized path on top.
    snprintf(plot, sizeof(plot),
             "set title \"%s - XY path\"\n"
             "plot $poly using 1:2 with lines linewidth 2 lc rgb 'grey' "
             "title \"commanded\", $data using 2:3 with linespoints "
             "linewidth 1 title \"realized\"\n",
             title);
    fprintf(gp, "%s", plot);

    // Panel 2: per-axis speed over time (one series per axis).
    fprintf(gp, "set title \"speed [steps/s] over time [s]\"\n");
    int cols2[1] = {5};
    int per2[1] = {1};
    const char* prefixes2[1] = {"speed"};
    build_statement(plot, sizeof(plot), cols2, per2, prefixes2, 1);
    fprintf(gp, "%s", plot);

    // Panel 3: performed (P) vs remaining (R) ramp steps over time, both
    // groups in one panel so P and R read together.
    fprintf(gp, "set title \"P vs R [steps] over time [s]\"\n");
    int cols3[2] = {6, 7};
    int per3[2] = {1, 1};
    const char* prefixes3[2] = {"P", "R"};
    build_statement(plot, sizeof(plot), cols3, per3, prefixes3, 2);
    fprintf(gp, "%s", plot);

    // Panel 4: per-axis period over time.
    fprintf(gp, "set title \"period [ticks] over time [s]\"\n");
    int cols4[1] = {8};
    int per4[1] = {1};
    const char* prefixes4[1] = {"period"};
    build_statement(plot, sizeof(plot), cols4, per4, prefixes4, 1);
    fprintf(gp, "%s", plot);

    // Panel 5: step deviation — commanded (x,y) minus realized (x,y) as a
    // single radius in steps, on its own scale. The XY panel above is too
    // coarse to reveal small per-step deviations.
    fprintf(gp, "set yrange [-10:10]\n");
    fprintf(gp, "set title \"step deviation [steps] over time [s]\"\n");
    int cols5[1] = {4};
    int per5[1] = {0};
    const char* prefixes5[1] = {"dev"};
    build_statement(plot, sizeof(plot), cols5, per5, prefixes5, 1);
    fprintf(gp, "%s", plot);

    fclose(gp);
    gp = NULL;
    open = false;
  }

  bool is_open() const { return open; }

 private:
  char filename[100];
  char title[100];
  char plot[512];
  FILE* gp;
  bool open;

  // Emits one newline-terminated "plot <series>, <series> ..." statement into
  // `out`. Each (col, prefix) group contributes one series per axis
  // ("using 1:<col> with linespoints title \"<prefix><i>\""), all groups
  // comma-chained into a single plot statement so they share one panel. When
  // `per_axis[g]` is set the column is offset by 4*i so the per-axis overlay
  // series land in their own columns; deviation is a single column (per_axis
  // 0). `len` accumulates because snprintf returns the chars that *would* be
  // written, not the running total.
  void build_statement(char* out, size_t out_sz, const int* cols,
                       const int* per_axis, const char** prefixes,
                       int ngroups) {
    size_t len = 0;
    bool first = true;
    for (int g = 0; g < ngroups; g++) {
      for (int i = 0; i < n_axes; i++) {
        int col = per_axis[g] ? cols[g] + 4 * i : cols[g];
        int w = snprintf(out + len, out_sz - len,
                         "%s$data using 1:%d with linespoints "
                         "title \"%s%d\"",
                         first ? "plot " : ", ", col, prefixes[g], i);
        first = false;
        if (w < 0 || (size_t)w >= out_sz - len) {
          out[0] = 0;
          return;
        }
        len += (size_t)w;
      }
    }
    out[len++] = '\n';
    out[len] = 0;
  }
};

#endif /* NAXIS_PLOT_H */
