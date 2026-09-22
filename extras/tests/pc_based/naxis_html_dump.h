// naxis_html_dump.h — optional HTML trace viewer for the FasNAxis PC tests.
//
// Test-only. Not included from src/FasNAxis.h; pulled in by test_26.cpp only
// under -DFAS_NAXIS_TRACE. It copies the checked-in static page
// extras/n_axes/viewer_template.html (a <pre id="trace"> placeholder) and
// embeds the same (t, x, y) samples the gnuplot file already carries, writing
// extras/n_axes/tests/out/<fixture>.html. Without the macro the file is not
// created and src/FasNAxis.h has no viewer include.
//
// Protocol:
//   NaxisHtmlDump html("F5", "FasNAxis F5 square 1600 Linear");
//   for (sample s in run) html.row(s.t, s.x, s.y);
//   html.finish();
//
#ifndef NAXIS_HTML_DUMP_H
#define NAXIS_HTML_DUMP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Root of the checked-in viewer assets. The trace build passes an absolute
// path (-DNAXIS_HTML_ROOT="$(PRJ_ROOT)/extras/n_axes") so the page resolves
// regardless of the test's working directory; the default is the repo-relative
// path for a manual run from the repo root.
#ifndef NAXIS_HTML_ROOT
#define NAXIS_HTML_ROOT "extras/n_axes"
#endif

class NaxisHtmlDump {
 public:
  NaxisHtmlDump(const char* fixture, const char* title)
      : template_buf(NULL),
        rows_buf(NULL),
        rows_len(0),
        n_rows(0),
        open(false) {
    snprintf(out_path, sizeof(out_path), "%s/tests/out/%s.html",
             NAXIS_HTML_ROOT, fixture);
    snprintf(title_buf, sizeof(title_buf), "%s", title);
    rows_buf = (char*)malloc(NAXIS_HTML_MAX_BYTES);
    if (rows_buf == NULL) {
      return;
    }
    char tpl_path[256];
    snprintf(tpl_path, sizeof(tpl_path), "%s/viewer_template.html",
             NAXIS_HTML_ROOT);
    FILE* tpl = fopen(tpl_path, "r");
    if (tpl == NULL) {
      return;
    }
    size_t len = 0;
    fseek(tpl, 0, SEEK_END);
    len = ftell(tpl);
    fseek(tpl, 0, SEEK_SET);
    if (len > 0) {
      template_buf = (char*)malloc(len + 1);
      if (template_buf != NULL) {
        size_t got = fread(template_buf, 1, len, tpl);
        template_buf[got] = 0;
        open = true;
      }
    }
    fclose(tpl);
  }

  // One (t, x, y) row, the same columns the gnuplot data heredoc carries.
  void row(double t, double x, double y) {
    if (!open || n_rows >= NAXIS_HTML_MAX_ROWS) {
      return;
    }
    char line[128];
    int w = snprintf(line, sizeof(line), "%.6f %.6f %.6f\n", t, x, y);
    if (w < 0 || (size_t)w >= sizeof(line)) {
      return;
    }
    if (rows_len + (size_t)w > NAXIS_HTML_MAX_BYTES) {
      return;
    }
    memcpy(rows_buf + rows_len, line, (size_t)w);
    rows_len += (size_t)w;
    n_rows++;
  }

  // Splice the accumulated rows into the template's <pre id="trace"> and write
  // the page. A missing template or output dir leaves nothing on disk.
  void finish() {
    if (!open) {
      return;
    }
    const char* open_tag = "<pre id=\"trace\">";
    const char* close_tag = "</pre>";
    char* at = strstr(template_buf, open_tag);
    if (at == NULL) {
      return;
    }
    char* after = at + strlen(open_tag);
    char* end = strstr(after, close_tag);
    if (end == NULL) {
      return;
    }
    size_t head = (size_t)(after - template_buf);
    size_t tail = strlen(end);
    size_t need = head + rows_len + tail + 1;
    char* out = (char*)malloc(need);
    if (out == NULL) {
      return;
    }
    memcpy(out, template_buf, head);
    memcpy(out + head, rows_buf, rows_len);
    memcpy(out + head + rows_len, end, tail);
    out[head + rows_len + tail] = 0;

    FILE* f = fopen(out_path, "w");
    if (f != NULL) {
      fwrite(out, 1, head + rows_len + tail, f);
      fclose(f);
    }
    free(out);
    free(template_buf);
    template_buf = NULL;
    open = false;
  }

  ~NaxisHtmlDump() {
    free(template_buf);
    free(rows_buf);
  }

  int rows_written() const { return n_rows; }

 private:
  static const int NAXIS_HTML_MAX_ROWS = 200000;
  static const size_t NAXIS_HTML_MAX_BYTES = (size_t)NAXIS_HTML_MAX_ROWS * 128;
  char out_path[128];
  char title_buf[128];
  char* template_buf;
  char* rows_buf;
  size_t rows_len;
  int n_rows;
  bool open;
};

#endif /* NAXIS_HTML_DUMP_H */
