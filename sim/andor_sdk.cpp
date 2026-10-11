#include "atmcdLXd.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

static int g_coolerOn = 0;
static int g_targetTemp = -50;
static int g_currentTemp = 20;
static const int AMBIENT_TEMP = 20;
static const int TEMP_STEP = 35;

extern "C" {
unsigned int Initialize(char *) {
  return DRV_SUCCESS;
}

unsigned int ShutDown() {
  return DRV_SUCCESS;
}

unsigned int GetStatus(int *s) {
  *s = DRV_IDLE;
  return DRV_SUCCESS;
}

unsigned int GetTemperature(int *t) {
  if (g_coolerOn) {
    if (g_currentTemp > g_targetTemp) {
      g_currentTemp -= TEMP_STEP;
      if (g_currentTemp < g_targetTemp)
        g_currentTemp = g_targetTemp;
    }
  } else {
    if (g_currentTemp < AMBIENT_TEMP) {
      g_currentTemp += TEMP_STEP;
      if (g_currentTemp > AMBIENT_TEMP)
        g_currentTemp = AMBIENT_TEMP;
    }
  }
  *t = g_currentTemp;
  if (g_currentTemp == g_targetTemp && g_coolerOn)
    return DRV_TEMP_STABILIZED;
  if (!g_coolerOn)
    return DRV_TEMP_OFF;
  return DRV_TEMP_NOT_REACHED;
}

unsigned int GetTemperatureF(float *t) {
  int it;
  unsigned int res = GetTemperature(&it);
  *t = (float)it;
  return res;
}

unsigned int SetTemperature(int t) {
  g_targetTemp = t;
  return DRV_SUCCESS;
}

unsigned int CoolerON() {
  g_coolerOn = 1;
  return DRV_SUCCESS;
}

unsigned int CoolerOFF() {
  g_coolerOn = 0;
  g_currentTemp = 20;
  return DRV_SUCCESS;
}

unsigned int IsCoolerOn(int *s) {
  *s = g_coolerOn;
  return DRV_SUCCESS;
}

int select_camera(int) noexcept {
  return DRV_SUCCESS;
}

unsigned int GetNumberHSSpeeds(int, int, int *s) {
  *s = 1;
  return DRV_SUCCESS;
}

unsigned int GetHSSpeed(int, int, int, float *s) {
  *s = 10.0;
  return DRV_SUCCESS;
}

unsigned int GetPreAmpGain(int, float *g) {
  *g = 1.0;
  return DRV_SUCCESS;
}

unsigned int GetAcquisitionTimings(float *e, float *a, float *k) {
  *e = 2.0;
  *a = 2.0;
  *k = 2.0;
  return DRV_SUCCESS;
}

unsigned int GetReadOutTime(float *r) {
  *r = 0.1;
  return DRV_SUCCESS;
}

unsigned int GetAvailableCameras(at_32 *t) {
  *t = 1;
  return DRV_SUCCESS;
}

unsigned int GetCameraHandle(at_32, at_32 *p) {
  *p = 1;
  return DRV_SUCCESS;
}

unsigned int SetCurrentCamera(at_32) {
  return DRV_SUCCESS;
}

unsigned int SetReadMode(int) {
  return DRV_SUCCESS;
}

unsigned int SetSingleTrack(int, int) {
  return DRV_SUCCESS;
}

unsigned int SetImage(int, int, int, int, int, int) {
  return DRV_SUCCESS;
}

unsigned int SetAcquisitionMode(int) {
  return DRV_SUCCESS;
}

unsigned int SetExposureTime(float) {
  return DRV_SUCCESS;
}

unsigned int SetVSSpeed(int) {
  return DRV_SUCCESS;
}

unsigned int SetHSSpeed(int, int) {
  return DRV_SUCCESS;
}

unsigned int SetPreAmpGain(int) {
  return DRV_SUCCESS;
}

unsigned int GetNumberVSSpeeds(int *s) {
  *s = 1;
  return DRV_SUCCESS;
}

unsigned int GetVSSpeed(int, float *s) {
  *s = 1.0;
  return DRV_SUCCESS;
}

unsigned int GetDetector(int *x, int *y) {
  *x = 2048;
  *y = 2048;
  return DRV_SUCCESS;
}

unsigned int GetAcquiredData(at_32 *, unsigned long) {
  return DRV_SUCCESS;
}

unsigned int AbortAcquisition() {
  return DRV_SUCCESS;
}

unsigned int StartAcquisition() {
  return DRV_SUCCESS;
}

unsigned int WaitForAcquisition() {
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  return DRV_SUCCESS;
}

unsigned int CancelWait() {
  return DRV_SUCCESS;
}

unsigned int GetNumberNewImages(at_32 *f, at_32 *l) {
  *f = 1;
  *l = 1;
  return DRV_SUCCESS;
}

unsigned int GetTotalNumberImagesAcquired(at_32 *i) {
  *i = 1;
  return DRV_SUCCESS;
}

unsigned int GetMostRecentImage(at_32 *a, unsigned long s) {
  return DRV_SUCCESS;
}

unsigned int GetImages(at_32 f, at_32 l, at_32 *a, unsigned long s, at_32 *vf,
                       at_32 *vl) {
  *vf = 1;
  *vl = 1;
  return DRV_SUCCESS;
}

unsigned int SetNumberAccumulations(int) {
  return DRV_SUCCESS;
}

unsigned int SetAccumulationCycleTime(float) {
  return DRV_SUCCESS;
}

unsigned int SetNumberKinetics(int) {
  return DRV_SUCCESS;
}

unsigned int SetKineticCycleTime(float) {
  return DRV_SUCCESS;
}

unsigned int SetTriggerMode(int) {
  return DRV_SUCCESS;
}

unsigned int SetShutter(int, int, int, int) {
  return DRV_SUCCESS;
}

unsigned int GetHeadModel(char *m) {
  std::strcpy(m, "MOCK");
  return DRV_SUCCESS;
}

unsigned int GetCameraSerialNumber(int *s) {
  *s = 1;
  return DRV_SUCCESS;
}

unsigned int GetTemperatureRange(int *mn, int *mx) {
  *mn = -100;
  *mx = 20;
  return DRV_SUCCESS;
}

unsigned int GetNumberPreAmpGains(int *n) {
  *n = 1;
  return DRV_SUCCESS;
}

unsigned int GetNumberAmp(int *a) {
  *a = 1;
  return DRV_SUCCESS;
}

unsigned int GetNumberADChannels(int *c) {
  *c = 1;
  return DRV_SUCCESS;
}

unsigned int GetCurrentPreAmpGain(int *i, char *n, int) {
  *i = 0;
  std::strcpy(n, "MOCK");
  return DRV_SUCCESS;
}

unsigned int GetFastestRecommendedVSSpeed(int *i, float *s) {
  *i = 0;
  *s = 1.0;
  return DRV_SUCCESS;
}

unsigned int IsPreAmpGainAvailable(int, int, int, int, int *s) {
  *s = 1;
  return DRV_SUCCESS;
}

unsigned int GetAmpDesc(int, char *n, int) {
  std::strcpy(n, "MOCK");
  return DRV_SUCCESS;
}
}
