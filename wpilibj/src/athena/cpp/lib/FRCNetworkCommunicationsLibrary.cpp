/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <assert.h>
#include <jni.h>
#include "Log.h"

#include "HAL/HAL.h"
#include "edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary.h"
#include "HALUtil.h"

// set the logging level
TLogLevel netCommLogLevel = logWARNING;

#define NETCOMM_LOG(level)     \
  if (level > netCommLogLevel) \
    ;                          \
  else                         \
  Log().Get(level)

extern "C" {

/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_nUsageReporting_report
 * Signature: (BBBLjava/lang/String;)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationUsageReportingReport(
    JNIEnv *paramEnv, jclass, jbyte paramResource, jbyte paramInstanceNumber,
    jbyte paramContext, jstring paramFeature) {
  const char *featureStr = paramEnv->GetStringUTFChars(paramFeature, nullptr);
  NETCOMM_LOG(logDEBUG) << "Calling FRCNetworkCommunicationsLibrary report "
                        << "res:" << (unsigned int)paramResource
                        << " instance:" << (unsigned int)paramInstanceNumber
                        << " context:" << (unsigned int)paramContext
                        << " feature:" << featureStr;
  jint returnValue =
      HAL_Report(paramResource, paramInstanceNumber, paramContext, featureStr);
  paramEnv->ReleaseStringUTFChars(paramFeature, featureStr);
  return returnValue;
}
/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_observeUserProgramStarting
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationObserveUserProgramStarting(
    JNIEnv *, jclass) {
  HAL_NetworkCommunicationObserveUserProgramStarting();
}
/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_observeUserProgramDisabled
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationObserveUserProgramDisabled(
    JNIEnv *, jclass) {
  HAL_NetworkCommunicationObserveUserProgramDisabled();
}
/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_observeUserProgramAutonomous
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationObserveUserProgramAutonomous(
    JNIEnv *, jclass) {
  HAL_NetworkCommunicationObserveUserProgramAutonomous();
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_observeUserProgramTeleop
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationObserveUserProgramTeleop(
    JNIEnv *, jclass) {
  HAL_NetworkCommunicationObserveUserProgramTeleop();
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRC_NetworkCommunicationsLibrary
 * Method:    FRC_NetworkCommunication_observeUserProgramTest
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationObserveUserProgramTest(
    JNIEnv *, jclass) {
  HAL_NetworkCommunicationObserveUserProgramTest();
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    FRCNetworkCommunicationReserve
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_FRCNetworkCommunicationReserve(
    JNIEnv *, jclass) {
  int rv = HAL_Initialize(0);
  assert(1 == rv);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    NativeHALGetControlWord
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_NativeHALGetControlWord(
    JNIEnv *, jclass) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL Control Word";
  jint controlWord;
  HAL_GetControlWord((HAL_ControlWord *)&controlWord);
  return controlWord;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    NativeHALGetAllianceStation
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_NativeHALGetAllianceStation(
    JNIEnv *, jclass) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL Alliance Station";
  jint allianceStation;
  HAL_GetAllianceStation((HAL_AllianceStationID *)&allianceStation);
  return allianceStation;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickAxes
 * Signature: (B[S)B
 */
JNIEXPORT jbyte JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickAxes(
    JNIEnv * env, jclass, jbyte joystickNum, jfloatArray axesArray) {
  NETCOMM_LOG(logDEBUG) << "Calling HALJoystickAxes";
  HAL_JoystickAxes axes;
  HAL_GetJoystickAxes(joystickNum, &axes);

  jsize javaSize = env->GetArrayLength(axesArray);
  if (axes.count > javaSize)
  {
    ThrowIllegalArgumentException(env, "Native array size larger then passed in java array size");
  }

  env->SetFloatArrayRegion(axesArray, 0, axes.count, axes.axes);

  return axes.count;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickPOVs
 * Signature: (B[S)B
 */
JNIEXPORT jbyte JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickPOVs(
    JNIEnv * env, jclass, jbyte joystickNum, jshortArray povsArray) {
  NETCOMM_LOG(logDEBUG) << "Calling HALJoystickPOVs";
  HAL_JoystickPOVs povs;
  HAL_GetJoystickPOVs(joystickNum, &povs);

  jsize javaSize = env->GetArrayLength(povsArray);
  if (povs.count > javaSize)
  {
    ThrowIllegalArgumentException(env, "Native array size larger then passed in java array size");
  }

  env->SetShortArrayRegion(povsArray, 0, povs.count, povs.povs);

  return povs.count;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickButtons
 * Signature: (B)S
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickButtons(
    JNIEnv *env, jclass, jbyte joystickNum, jobject count) {
  NETCOMM_LOG(logDEBUG) << "Calling HALJoystickButtons";
  HAL_JoystickButtons joystickButtons;
  HAL_GetJoystickButtons(joystickNum, &joystickButtons);
  jbyte *countPtr = (jbyte *)env->GetDirectBufferAddress(count);
  NETCOMM_LOG(logDEBUG) << "Buttons = " << joystickButtons.buttons;
  NETCOMM_LOG(logDEBUG) << "Count = " << (jint)joystickButtons.count;
  *countPtr = joystickButtons.count;
  NETCOMM_LOG(logDEBUG) << "CountBuffer = " << (jint)*countPtr;
  return joystickButtons.buttons;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_SetJoystickOutputs
 * Signature: (BISS)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALSetJoystickOutputs(
    JNIEnv *, jclass, jbyte port, jint outputs, jshort leftRumble,
    jshort rightRumble) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL_SetJoystickOutputs on port " << port;
  NETCOMM_LOG(logDEBUG) << "Outputs: " << outputs;
  NETCOMM_LOG(logDEBUG) << "Left Rumble: " << leftRumble
                        << " Right Rumble: " << rightRumble;
  return HAL_SetJoystickOutputs(port, outputs, leftRumble, rightRumble);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickIsXbox
 * Signature: (B)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickIsXbox(
    JNIEnv *, jclass, jbyte port) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL_GetJoystickIsXbox";
  return HAL_GetJoystickIsXbox(port);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickType
 * Signature: (B)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickType(
    JNIEnv *, jclass, jbyte port) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL_GetJoystickType";
  return HAL_GetJoystickType(port);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickName
 * Signature: (B)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickName(
    JNIEnv *env, jclass, jbyte port) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL_GetJoystickName";
  char *joystickName = HAL_GetJoystickName(port);
  jstring str = env->NewStringUTF(joystickName);
  std::free(joystickName);
  return str;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetJoystickAxisType
 * Signature: (BB)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetJoystickAxisType(
    JNIEnv *, jclass, jbyte joystickNum, jbyte axis) {
  NETCOMM_LOG(logDEBUG) << "Calling HAL_GetJoystickAxisType";
  return HAL_GetJoystickAxisType(joystickNum, axis);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    setNewDataSem
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_setNewDataSem(
    JNIEnv *env, jclass, jlong id) {
  NETCOMM_LOG(logDEBUG) << "Mutex Ptr = " << (void *)id;
  HAL_SetNewDataSem((MULTIWAIT_ID)id);
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary

 * Method:    HAL_GetMatchTime
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetMatchTime(
    JNIEnv *env, jclass) {
  jfloat matchTime;
  HAL_GetMatchTime((float *)&matchTime);
  return matchTime;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetSystemActive
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetSystemActive(
    JNIEnv *env, jclass) {
  int32_t status = 0;
  bool val = HAL_GetSystemActive(&status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_GetBrownedOut
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALGetBrownedOut(
    JNIEnv *env, jclass) {
  int32_t status = 0;
  bool val = HAL_GetBrownedOut(&status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_SetErrorData
 * Signature: (Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALSetErrorData(
    JNIEnv *env, jclass, jstring error) {
  const char *errorStr = env->GetStringUTFChars(error, nullptr);
  jsize length = env->GetStringUTFLength(error);

  NETCOMM_LOG(logDEBUG) << "Set Error: " << errorStr;
  NETCOMM_LOG(logDEBUG) << "Length: " << length;
  jint returnValue = HAL_SetErrorData(errorStr, (jint)length, 0);
  env->ReleaseStringUTFChars(error, errorStr);
  return returnValue;
}

/*
 * Class: edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary
 * Method:    HAL_SendError
 * Signature: (ZIZLjava/lang/String;Ljava/lang/String;Ljava/lang/String;Z)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_wpilibj_communication_FRCNetworkCommunicationsLibrary_HALSendError(
    JNIEnv *env, jclass, jboolean isError, jint errorCode, jboolean isLVCode,
    jstring details, jstring location, jstring callStack, jboolean printMsg) {
  const char *detailsStr = env->GetStringUTFChars(details, nullptr);
  const char *locationStr = env->GetStringUTFChars(location, nullptr);
  const char *callStackStr = env->GetStringUTFChars(callStack, nullptr);

  NETCOMM_LOG(logDEBUG) << "Send Error: " << detailsStr;
  NETCOMM_LOG(logDEBUG) << "Location: " << locationStr;
  NETCOMM_LOG(logDEBUG) << "Call Stack: " << callStackStr;
  jint returnValue = HAL_SendError(isError, errorCode, isLVCode, detailsStr,
                                  locationStr, callStackStr, printMsg);
  env->ReleaseStringUTFChars(details, detailsStr);
  env->ReleaseStringUTFChars(location, locationStr);
  env->ReleaseStringUTFChars(callStack, callStackStr);
  return returnValue;
}

}  // extern "C"
