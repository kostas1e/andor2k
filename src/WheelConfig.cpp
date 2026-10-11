#include "WheelConfig.hpp"

#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace andor2k {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Strip leading and trailing ASCII whitespace from s in-place.
/// Returns pointer to first non-whitespace character.
static char *trim_inplace(char *s) {
  while (*s == ' ' || *s == '\t')
    ++s;
  char *end = s + static_cast<int>(std::strlen(s)) - 1;
  while (end > s &&
         (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r'))
    *end-- = '\0';
  return s;
}

/// Truncate at first '#' (inline comment).
static void strip_comment(char *s) {
  char *p = s;
  while (*p && *p != '#')
    ++p;
  *p = '\0';
}

// ---------------------------------------------------------------------------
// WheelConfig::load
// ---------------------------------------------------------------------------

bool WheelConfig::load(const char *path) {
  FILE *f = std::fopen(path, "r");
  if (!f) {
    // ENOENT is expected (file absent = use defaults). Other errors are real.
    if (errno != ENOENT) {
      printf("[WheelConfig] Cannot open '%s': %s\n", path,
             std::strerror(errno));
    }
    return false;
  }

  printf("[WheelConfig] Loading '%s'\n", path);

  char line[256];
  int lineno = 0;

  while (std::fgets(line, static_cast<int>(sizeof(line)), f)) {
    ++lineno;
    strip_comment(line);
    char *p = trim_inplace(line);
    if (*p == '\0')
      continue; // blank after stripping

    // Parse "key = value"
    char key[64] = {0};
    char val[64] = {0};
    if (std::sscanf(p, "%63[^ =\t] = %63s", key, val) != 2) {
      printf(
          "[WheelConfig] %s:%d: bad format (expected 'key = value'), skipped\n",
          path, lineno);
      continue;
    }

    // Helpers: parse value and assign, logging on overflow/bad parse.
    auto set_long = [&](long &field, long lo, long hi) {
      char *end = nullptr;
      errno = 0;
      long v = std::strtol(val, &end, 10);
      if (errno != 0 || end == val || *end != '\0') {
        printf("[WheelConfig] %s:%d: cannot parse '%s' as integer, skipped\n",
               path, lineno, val);
        return;
      }
      if (v < lo || v > hi) {
        printf(
            "[WheelConfig] %s:%d: value %ld out of range [%ld, %ld], skipped\n",
            path, lineno, v, lo, hi);
        return;
      }
      field = v;
    };
    auto set_uint = [&](unsigned int &field, unsigned int lo, unsigned int hi) {
      char *end = nullptr;
      errno = 0;
      unsigned long v = std::strtoul(val, &end, 10);
      if (errno != 0 || end == val || *end != '\0') {
        printf("[WheelConfig] %s:%d: cannot parse '%s' as unsigned integer, "
               "skipped\n",
               path, lineno, val);
        return;
      }
      if (v < static_cast<unsigned long>(lo) ||
          v > static_cast<unsigned long>(hi)) {
        printf(
            "[WheelConfig] %s:%d: value %lu out of range [%u, %u], skipped\n",
            path, lineno, v, lo, hi);
        return;
      }
      field = static_cast<unsigned int>(v);
    };

    // Match keys
    if (!std::strcmp(key, "qc_per_rotation"))
      set_long(qc_per_rotation, 1000000L, 20000000L);
    else if (!std::strcmp(key, "lower_sensor_offset_qc"))
      set_long(lower_sensor_offset_qc, 0L, 20000000L);
    else if (!std::strcmp(key, "upper_sensor_offset_qc"))
      set_long(upper_sensor_offset_qc, 0L, 20000000L);
    else if (!std::strcmp(key, "max_drift_qc"))
      set_long(max_drift_qc, 1000L, 5000000L);
    else if (!std::strcmp(key, "slot_tolerance_qc"))
      set_long(slot_tolerance_qc, 1000L, 2000000L);
    else if (!std::strcmp(key, "homing_gap_max_qc"))
      set_long(homing_gap_max_qc, 10000L, 5000000L);
    else if (!std::strcmp(key, "profile_velocity"))
      set_uint(profile_velocity, 500, 10000);
    else if (!std::strcmp(key, "homing_velocity"))
      set_uint(homing_velocity, 500, 10000);
    else if (!std::strcmp(key, "profile_acceleration"))
      set_uint(profile_acceleration, 1000, 100000);
    else if (!std::strcmp(key, "profile_deceleration"))
      set_uint(profile_deceleration, 1000, 100000);
    else
      printf("[WheelConfig] %s:%d: unknown key '%s', skipped\n", path, lineno,
             key);
  }

  std::fclose(f);
  loaded = true;
  return true;
}

// ---------------------------------------------------------------------------
// WheelConfig::print
// ---------------------------------------------------------------------------

void WheelConfig::print(const char *label) const {
  printf("[%s] %s\n", label,
         loaded ? "(loaded from file)" : "(compiled defaults)");
  printf("  qc_per_rotation         = %ld\n", qc_per_rotation);
  printf("  lower_sensor_offset_qc  = %ld\n", lower_sensor_offset_qc);
  printf("  upper_sensor_offset_qc  = %ld\n", upper_sensor_offset_qc);
  printf("  max_drift_qc            = %ld\n", max_drift_qc);
  printf("  slot_tolerance_qc       = %ld\n", slot_tolerance_qc);
  printf("  homing_gap_max_qc       = %ld\n", homing_gap_max_qc);
  printf("  profile_velocity        = %u rpm\n", profile_velocity);
  printf("  homing_velocity         = %u rpm\n", homing_velocity);
  printf("  profile_acceleration    = %u rpm/s\n", profile_acceleration);
  printf("  profile_deceleration    = %u rpm/s\n", profile_deceleration);
}

} // namespace andor2k
