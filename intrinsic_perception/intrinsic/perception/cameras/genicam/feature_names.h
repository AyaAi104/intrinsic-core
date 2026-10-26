// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_PERCEPTION_CAMERAS_GENICAM_FEATURE_NAMES_H_
#define INTRINSIC_PERCEPTION_CAMERAS_GENICAM_FEATURE_NAMES_H_

namespace intrinsic {
namespace perception {
namespace genicam {

// All parameters and their names listed in this file belong to the GenICam
// "Standard Features Naming Convention" (SFNC). The latest version can be found
// here: https://www.emva.org/wp-content/uploads/GenICam_SFNC_v2_7.pdf.

// Device parameters

// Name of the manufacturer of the device.
inline constexpr char kDeviceVendorName[] = "DeviceVendorName";

// Model of the device.
inline constexpr char kDeviceModelName[] = "DeviceModelName";

// Device's serial number.
inline constexpr char kDeviceSerialNumber[] = "DeviceSerialNumber";

// Firmware version in the device.
inline constexpr char kDeviceFirmwareVersion[] = "DeviceFirmwareVersion";

// Major version of the Standard Features Naming Convention that was used to
// create the device's GenICam XML.
inline constexpr char kDeviceSFNCVersionMajor[] = "DeviceSFNCVersionMajor";

// Minor version of the Standard Features Naming Convention that was used to
// create the device's GenICam XML.
inline constexpr char kDeviceSFNCVersionMinor[] = "DeviceSFNCVersionMinor";

// Sub minor version of Standard Features Naming Convention that was used to
// create the device's GenICam XML.
inline constexpr char kDeviceSFNCVersionSubMinor[] =
    "DeviceSFNCVersionSubMinor";

// Resets the device to its power up state. After reset, the device must be
// rediscovered.
inline constexpr char kDeviceReset[] = "DeviceReset";

// Image parameters

// Width of the image provided by the device (in pixels).
// This reflects the current Region of interest. The maximum value of this
// feature takes into account horizontal binning, decimation, or any other
// function changing the maximum horizontal dimensions of the image and is
// typically equal to WidthMax - OffsetX. This feature is generally mandatory
// for transmitters and transceivers of most Transport Layers.
inline constexpr char kWidth[] = "Width";

// Height of the image provided by the device (in pixels).
// This reflects the current Region of interest. The maximum value of this
// feature takes into account vertical binning, decimation, or any other
// function changing the maximum vertical dimensions of the image and is
// typically equal to HeightMax - OffsetY. This feature is generally mandatory
// for transmitters and transceivers of most Transport Layers.
inline constexpr char kHeight[] = "Height";

// Horizontal offset from the origin to the region of interest (in pixels).
inline constexpr char kOffsetX[] = "OffsetX";

// Vertical offset from the origin to the region of interest (in pixels).
inline constexpr char kOffsetY[] = "OffsetY";

// Effective width of the sensor in pixels.
inline constexpr char kSensorWidth[] = "SensorWidth";

// Effective height of the sensor in pixels.
inline constexpr char kSensorHeight[] = "SensorHeight";

// Selects a component to activate/deactivate its data streaming.
inline constexpr char kComponentSelector[] = "ComponentSelector";

// Controls if the selected component streaming is active.
inline constexpr char kComponentEnable[] = "ComponentEnable";

// Returns a unique identifier corresponding to the selected component.
inline constexpr char kComponentIDValue[] = "ComponentIDValue";

// Format of the pixels provided by the device. It represents all the
// information provided by PixelSize, PixelColorFilter combined in a single
// feature.
inline constexpr char kPixelFormat[] = "PixelFormat";

// Total size in bits of a pixel of the image.
inline constexpr char kPixelSize[] = "PixelSize";

// Provides the number of bytes transferred for each data buffer or chunk on the
// stream channel.
inline constexpr char kPayloadSize[] = "PayloadSize";

// Acquisition parameters

// Selects the type of trigger to configure.
inline constexpr char kTriggerSelector[] = "TriggerSelector";

// Controls if the selected trigger is active.
inline constexpr char kTriggerMode[] = "TriggerMode";

// Specifies the internal signal or physical input line to use as the trigger
// source. The selected trigger must have its TriggerMode set to On.
inline constexpr char kTriggerSource[] = "TriggerSource";

// Specifies the activation mode of the trigger.
inline constexpr char kTriggerActivation[] = "TriggerActivation";

// Sets the acquisition mode of the device.
inline constexpr char kAcquisitionMode[] = "AcquisitionMode";

// Specifies the shutter mode of the device.
inline constexpr char kSensorShutterMode[] = "SensorShutterMode";

// Controls the gamma correction of pixel intensity.
inline constexpr char kGamma[] = "Gamma";

// Controls if the AcquisitionFrameRate feature is writable and used to control
// the acquisition rate.
inline constexpr char kAcquisitionFrameRateEnable[] =
    "AcquisitionFrameRateEnable";
inline constexpr char kAcquisitionFrameRateEnabled[] =
    "AcquisitionFrameRateEnabled";

// Controls the rate (in Hertz) at which the Lines in a Frame are captured.
inline constexpr char kAcquisitionFrameRate[] = "AcquisitionFrameRate";
inline constexpr char kAcquisitionFrameRateAbs[] = "AcquisitionFrameRateAbs";
inline constexpr char kFPS[] = "FPS";

// Selector for Gain.
inline constexpr char kGainSelector[] = "GainSelector";

// Common selector for GainSelector.
inline constexpr char kGainCommon[] = "Common";

// Infrared selector for GainSelector.
inline constexpr char kGainInfrared[] = "Infrared";

// Sets the automatic gain control (AGC) mode.
inline constexpr char kGainAuto[] = "GainAuto";

// Controls the selected gain as an absolute physical value.
inline constexpr char kGain[] = "Gain";
inline constexpr char kGainRaw[] = "GainRaw";
inline constexpr char kGainAbs[] = "GainAbs";

// Sets the operation mode of the Exposure.
inline constexpr char kExposureMode[] = "ExposureMode";

// Sets ExposureMode to Timed exposure.
inline constexpr char kExposureTimed[] = "Timed";

// Sets the automatic exposure mode when ExposureMode is Timed.
inline constexpr char kExposureAuto[] = "ExposureAuto";

// Sets ExposureAuto to Off.
inline constexpr char kExposureAutoOff[] = "Off";

// Sets ExposureAuto to Continuous.
inline constexpr char kExposureAutoContinuous[] = "Continuous";

// Sets the selector for ExposureTime.
inline constexpr char kExposureTimeSelector[] = "ExposureTimeSelector";

// Common selector for ExposureTime.
inline constexpr char kExposureTimeCommon[] = "Common";

// Infrared selector for ExposureTime.
inline constexpr char kExposureTimeInfrared[] = "Infrared";

// Sets the Exposure time (in microseconds) when ExposureMode is Timed and
// ExposureAuto is Off.
inline constexpr char kExposureTime[] = "ExposureTime";

// Deprecated SFNC attribute. Specified the exposure time of a camera in
// microseconds.
inline constexpr char kExposureTimeAbs[] = "ExposureTimeAbs";

// Deprecated SFNC attribute. Specified the exposure time of a camera in
// device specific units.
inline constexpr char kExposureTimeRaw[] = "ExposureTimeRaw";

// Deprecated SFNC attribute.
inline constexpr char kExposureTimeBaseAbs[] = "ExposureTimeBaseAbs";

// Controls automatic white balancing.
inline constexpr char kBalanceWhiteAuto[] = "BalanceWhiteAuto";

// Selector for BalanceRatio.
inline constexpr char kBalanceRatioSelector[] = "BalanceRatioSelector";

// BalanceRatioSelector selectors.
inline constexpr char kBalanceRatioAll[] = "All";
inline constexpr char kBalanceRatioRed[] = "Red";
inline constexpr char kBalanceRatioGreen[] = "Green";
inline constexpr char kBalanceRatioBlue[] = "Blue";

// Controls manual white balancing levels.
inline constexpr char kBalanceRatio[] = "BalanceRatio";

inline constexpr char kChunkSelector[] = "ChunkSelector";

inline constexpr char kChunkModeActive[] = "ChunkModeActive";

inline constexpr char kTimestamp[] = "Timestamp";

inline constexpr char kSensorPixelWidth[] = "SensorPixelWidth";
inline constexpr char kSensorPixelHeight[] = "SensorPixelHeight";

// Advanced parameters

inline constexpr char kScan3dFocalLength[] = "Scan3dFocalLength";
inline constexpr char kScan3dAspectRatio[] = "Scan3dAspectRatio";
inline constexpr char kScan3dPrincipalPointU[] = "Scan3dPrincipalPointU";
inline constexpr char kScan3dPrincipalPointV[] = "Scan3dPrincipalPointV";
inline constexpr char kScan3dDistanceUnit[] = "Scan3dDistanceUnit";

// Controls the delay (in GEV timestamp counter unit) to insert between each
// packet for this stream channel.
inline constexpr char kGevSCPD[] = "GevSCPD";

// Controls the data packet size transmitted via Ethernet.
inline constexpr char kGevSCPSPacketSize[] = "GevSCPSPacketSize";

// This feature is deprecated (See the increment of the TimestampLatchValue
// feature). It was used to indicate the number of timestamp ticks in 1 second
// (frequency in Hz). If PTP is used, this feature must return 1,000,000,000 (1
// GHz).
inline constexpr char kGevTimestampTickFrequency[] =
    "GevTimestampTickFrequency";

// The GevSCDMT parameter indicates the maximum amount of data (in bytes per
// second) that the camera is theoretically able to generate given its current
// settings and under ideal conditions without network restrictions.
inline constexpr char kGevSCDMT[] = "GevSCDMT";

// The GevSCDCT parameter indicates the actual bandwidth (in bytes per second)
// that the camera uses to transmit image data and chunk data given the current
// camera settings.
inline constexpr char kGevSCDCT[] = "GevSCDCT";

// The GevIEEE1588 parameter is used to enable the IEEE 1588 Precision Time
// Protocol (PTP) control the timestamp register. This parameter is deprecated
// and works for older camera models.
inline constexpr char kGevIEEE1588[] = "GevIEEE1588";

// Provides the state of the IEEE 1588 clock. This parameter is deprecated in
// favor of PtpStatus and works for older camera models.
inline constexpr char kGevIEEE1588Status[] = "GevIEEE1588Status";

// The PtpEnable parameter is used to enable the IEEE 1588 Precision Time
// Protocol (PTP) control the timestamp register. This parameter works for newer
// camera models.
inline constexpr char kPtpEnable[] = "PtpEnable";

// Latches the current values from the device's PTP clock data set.
inline constexpr char kPtpDataSetLatch[] = "PtpDataSetLatch";

// Returns the latched state of the PTP clock.
inline constexpr char kPtpStatus[] = "PtpStatus";

// HDR parameters

// Controls the state of multi-slope exposure. When set to "Off", the camera
// doesn't operate in multi-slope mode. For software HDR, use "SoftwareMertens".
inline constexpr char kMultiSlopeMode[] = "MultiSlopeMode";

// The number of knee-points to use for HDR. A knee-point signifies change in
// exposure. Number of knee-points should be 1 or more for HDR. N knee-points
// will result in N+1 underlying LDR exposures.
inline constexpr char kMultiSlopeKneePointCount[] = "MultiSlopeKneePointCount";

// Selects the knee-point.
inline constexpr char kMultiSlopeKneePointSelector[] =
    "MultiSlopeKneePointSelector";

// Sets the percent of the ExposureTime at the selected knee-point.
inline constexpr char kMultiSlopeExposureLimit[] = "MultiSlopeExposureLimit";

}  // namespace genicam
}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_GENICAM_FEATURE_NAMES_H_
