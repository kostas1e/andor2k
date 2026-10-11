#ifndef ATMCDLXD_H
#define ATMCDLXD_H

typedef int at_32;

#define DRV_SUCCESS 20002
#define DRV_VXDNOTINSTALLED 20003
#define DRV_ERROR_ACK 20004
#define DRV_INIERROR 20005
#define DRV_ACQUIRING 20072
#define DRV_IDLE 20073
#define DRV_TEMPCYCLE 20074
#define DRV_NOT_INITIALIZED 20075
#define DRV_P1INVALID 20066
#define DRV_P2INVALID 20067
#define DRV_P3INVALID 20068
#define DRV_P4INVALID 20069
#define DRV_P5INVALID 20070
#define DRV_P6INVALID 20071
#define DRV_NO_NEW_DATA 20008
#define DRV_GENERAL_ERRORS 20010
#define DRV_TEMP_OFF 20034
#define DRV_TEMP_NOT_REACHED 20035
#define DRV_TEMP_STABILIZED 20036
#define DRV_TEMP_NOT_STABILIZED 20037
#define DRV_TEMP_DRIFT 20040
#define DRV_SPOOLERROR 20043
#define DRV_SPOOLSETUPERROR 20044
#define DRV_BINNING_ERROR 20045
#define DRV_INVALID_FILTER 20046
#define DRV_ACCUM_TIME_NOT_MET 20047
#define DRV_KINETIC_TIME_NOT_MET 20048
#define DRV_ACQ_BUFFER 20049
#define DRV_ACQ_DOWNFIFO_FULL 20050
#define DRV_ERROR_PAGELOCK 20051
#define DRV_NOT_SUPPORTED 20052

#ifdef __cplusplus
extern "C" {
#endif
unsigned int Initialize(char *);
unsigned int ShutDown();
unsigned int GetStatus(int *);
unsigned int GetTemperature(int *);
unsigned int GetTemperatureF(float *);
unsigned int SetTemperature(int);
unsigned int CoolerON();
unsigned int CoolerOFF();
unsigned int IsCoolerOn(int *);
int select_camera(int) noexcept;
unsigned int GetNumberHSSpeeds(int, int, int *);
unsigned int GetHSSpeed(int, int, int, float *);
unsigned int GetPreAmpGain(int, float *);
unsigned int GetAcquisitionTimings(float *, float *, float *);
unsigned int GetReadOutTime(float *);
unsigned int GetAvailableCameras(at_32 *);
unsigned int GetCameraHandle(at_32, at_32 *);
unsigned int SetCurrentCamera(at_32);
unsigned int SetReadMode(int);
unsigned int SetSingleTrack(int, int);
unsigned int SetImage(int, int, int, int, int, int);
unsigned int SetAcquisitionMode(int);
unsigned int SetExposureTime(float);
unsigned int SetVSSpeed(int);
unsigned int SetHSSpeed(int, int);
unsigned int SetPreAmpGain(int);
unsigned int GetNumberVSSpeeds(int *);
unsigned int GetVSSpeed(int, float *);
unsigned int GetDetector(int *, int *);
unsigned int GetAcquiredData(at_32 *, unsigned long);
unsigned int AbortAcquisition();
unsigned int StartAcquisition();
unsigned int WaitForAcquisition();
unsigned int CancelWait();
unsigned int GetNumberNewImages(at_32 *, at_32 *);
unsigned int GetTotalNumberImagesAcquired(at_32 *);
unsigned int GetMostRecentImage(at_32 *, unsigned long);
unsigned int GetImages(at_32, at_32, at_32 *, unsigned long, at_32 *, at_32 *);
unsigned int GetHeadModel(char *);
unsigned int GetCameraSerialNumber(int *);
unsigned int GetTemperatureRange(int *, int *);
unsigned int GetNumberPreAmpGains(int *);
unsigned int GetNumberAmp(int *);
unsigned int GetNumberADChannels(int *);
unsigned int GetCurrentPreAmpGain(int *, char *, int);
unsigned int SetTriggerMode(int);
unsigned int SetNumberAccumulations(int);
unsigned int SetAccumulationCycleTime(float);
unsigned int SetNumberKinetics(int);
unsigned int SetKineticCycleTime(float);
unsigned int SetShutter(int, int, int, int);
unsigned int GetFastestRecommendedVSSpeed(int *, float *);
unsigned int IsPreAmpGainAvailable(int, int, int, int, int *);
unsigned int GetAmpDesc(int, char *, int);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ATMCDLXD_H
