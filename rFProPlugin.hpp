// Copyright (c) 2020 rFpro Limited, All Rights Reserved.
//
// NOTICE:  All information contained herein is, and remains the property of rFpro. The intellectual and technical concepts contained
// herein are proprietary to rFpro and may be covered by U.S. and foreign patents, patents in process, and are protected by trade secret or copyright law.
// Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained from rFpro.
//
// The copyright notice above does not evidence any actual or intended publication or disclosure of this source code, which includes information that is confidential
// and/or proprietary, and is a trade secret, of rFpro.  ANY REPRODUCTION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC DISPLAY OF THIS SOURCE CODE
// WITHOUT THE EXPRESS WRITTEN CONSENT OF RFPRO IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL TREATIES.
// THE RECEIPT OR POSSESSION OF THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO REPRODUCE,
// DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT MAY DESCRIBE, IN WHOLE OR IN PART.

#pragma once

#include "InternalsPluginExtras.hpp"
#include "TrafficDataModel.hpp"
#include "rFProTypes.h"

#define PLUGIN_NAME "rFProPlugin"

class rFProPlugin : public InternalsPluginV15Extras
{
public:
  rFProPlugin();
  virtual ~rFProPlugin();

  void Startup(long version) override;
  void Shutdown() override;

  void EnterRealtime() override;
  void ExitRealtime() override;

  void StartSession() override;
  void EndSession() override;

  void InitVehicle(const VehicleAndPhysicsV01 &data) override;

  // GAME OUTPUT
  long WantsTelemetryUpdates() override;
  void UpdateTelemetry( const TelemInfoV01 &info ) override;
  
  bool WantsGraphicsUpdates() override;
  void UpdateGraphics( const GraphicsInfoV02 &info ) override;

  void SetVehicleLocation( StartingVehicleLocationV01 &data ) override;

  long WantsTyreContactUpdates( long maxNumPoints, TyreContactPointDefnV01 *points ) override;
  void UpdateTyreContact( const TyreContactInputV01 &input ) override;
  bool WantsHDTyreContactUpdates( TyreDefinitionInputV01 &tyreDef ) override;
  void UpdateHDTyreContact( HDContactPatchV01 contactPatch[4] ) override;
  
  bool WantsMotionStates() override; 
  void UpdateMotionState(MotionPlatformV01 &motionInfo) override;

  void FMOD_ValidParameters( long numParams, const char **paramNames ) override;
  void FMOD_UpdateNew( long numParams, float *paramValues ) override;

  // Used to get traffic model for Insert Actor block
  void WantsDataModel(rFpro::DataModelProvider * provider) override;

  // Overrides for Additional Vehicle block (from plugin v2)
  virtual bool WantsToAddVehicle( const long id, NewVehicleDataV01 &data ) override;
  virtual bool GetVehicleState( VehicleStateV01 &data ) override;

  // LIGHT CONTROL
  bool WantsLightControl() override; // whether plugin wants light info and control
  bool GetLights( LightControlV01 &lightControl ) override;  // return true in order to control a single light; this will be called each frame continuously until it returns false
  bool UpdateSceneLights( SceneLightStatesV01 &state ) override; // return true for rFP to read new states

  // Object Proximity
  long WantsObjectProximityUpdates( long maxNumPoints, ProximityQueryPointDefnV01 *points ) override;
  void UpdateObjectProximity( const ProximityQueryInputV01 &input )  override;

  // Head tracking
  bool WantsToProvideHeadTracking() override;
  void UpdateHeadTrackingState(MotionPlatformV01 &motionInfo) override;
  
  // Camera view
  unsigned char WantsToViewVehicle(CameraControlInfoV01& camControl) override;
  
  // Bounding Box
  bool WantsBoundingBoxes() override;
  bool WantsBoundingBox(const QueryInstance& instance) override;
  void SetBoundingBoxes(long num, const BoundingBoxV01* boundingBoxArray) override;
  
  // Animation commands
  bool WantsAnimationInfo() { return true; }
  bool GetAnimation(AnimationActionV01& info) override;
  
#ifdef RUN_PHYSICS
  long GetPhysicsRate( long id ) override { return 400; }
  void RunPhysics( long id, PhysicsInputV01 &input ) override;
  void GetPhysicsState( long id, PhysicsOutputV01 &output ) override;
  bool ForceFeedback( double &forceValue ) override;
  bool CheckHWControl(const char * const controlName, double &fRetVal) override;
#endif

#ifdef TRAFFIC
  void SetSimulationTiming(const ThreadTimingV01 &info) override;
#endif
  
#ifdef SENSOR
  bool WantsSensorImageAccess() override { return true; }
  bool GetSensorImageSettings( SensorImageSettingsV01 &settings ) override;
  void SetSensorImage( const SensorImageInfoV01 &info ) override;
  void SetMultimediaTiming(const ThreadTimingV01 &info) override;
  long GetPreDrawParams( void *pData ) override;
#endif

#ifdef ADD_PHYSICS
  bool AddPhysics( const long id, double et, PhysicsAdditiveV01 &add ) override;
#endif

#ifdef COMPUTE_DIFF
  void InitDifferential( long id, DifferentialInitV01 &init ) override;
  bool ComputeDifferentialTransfer( long id, DifferentialInputV01 &input, DifferentialOutputV01 &output ) override;
#endif

#ifdef COMPUTE_TYRE
  void InitISOTyre( long id, ISOTyreInitV01 &init ) override;
  bool ComputeISOTyreForces( long id, ISOTyreInputV01 &input, ISOTyreOutputV01 &output ) override;
#endif
};
