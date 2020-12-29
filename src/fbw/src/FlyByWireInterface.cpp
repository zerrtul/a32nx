/*
 * A32NX
 * Copyright (C) 2020 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <INIReader.h>
#include <iomanip>
#include <iostream>

#include "FlyByWireInterface.h"
#include "SimConnectData.h"

using namespace std;

bool FlyByWireInterface::connect() {
  // register L variables for the sidestick
  idSideStickPositionX = register_named_variable("A32NX_SIDESTICK_POSITION_X");
  idSideStickPositionY = register_named_variable("A32NX_SIDESTICK_POSITION_Y");
  // register L variables for the sidestick on the left side
  idSideStickLeftPositionX = register_named_variable("A32NX_SIDESTICK_LEFT_POSITION_X");
  idSideStickLeftPositionY = register_named_variable("A32NX_SIDESTICK_LEFT_POSITION_Y");
  // register L variables for the sidestick on the right side
  idSideStickRightPositionX = register_named_variable("A32NX_SIDESTICK_RIGHT_POSITION_X");
  idSideStickRightPositionY = register_named_variable("A32NX_SIDESTICK_RIGHT_POSITION_Y");
  // register L variables for the rudder handling
  idRudderPositionOverrideOn = register_named_variable("A32NX_RUDDER_POSITION_OVERRIDE_ON");
  idRudderPosition = register_named_variable("A32NX_RUDDER_POSITION");
  // register L variables for the throttle handling
  idThrottlePositionOverrideOn = register_named_variable("A32NX_THROTTLE_POSITION_OVERRIDE_ON");
  idThrottlePosition_1 = register_named_variable("A32NX_THROTTLE_POSITION_1");
  idThrottlePosition_2 = register_named_variable("A32NX_THROTTLE_POSITION_2");

  // initialize throttle system
  initializeThrottles();

  // initialize model
  model.initialize();

  // initialize flight data recorder
  flightDataRecorder.initialize();

  // connect to sim connect
  return simConnectInterface.connect(isThrottleHandlingEnabled, idleThrottleInput, useReverseOnAxis);
}

void FlyByWireInterface::disconnect() {
  // disconnect from sim connect
  simConnectInterface.disconnect();

  // terminate model
  model.terminate();

  // terminate flight data recorder
  flightDataRecorder.terminate();
}

bool FlyByWireInterface::update(double sampleTime) {
  bool result = true;

  // get data & inputs
  result &= getModelInputDataFromSim(sampleTime);

  // step model
  model.step();

  // write output
  result &= writeModelOuputDataToSim();

  // get throttle data and process it
  if (isThrottleHandlingEnabled) {
    result &= processThrottles();
  }

  // update flight data recorder
  flightDataRecorder.update(&model);

  // return result
  return result;
}

bool FlyByWireInterface::getModelInputDataFromSim(double sampleTime) {
  // request data
  if (!simConnectInterface.requestData()) {
    cout << "WASM: Request data failed!" << endl;
    return false;
  }

  // read data
  if (!simConnectInterface.readData()) {
    cout << "WASM: Read data failed!" << endl;
    return false;
  }

  // get data from interface
  SimData simData = simConnectInterface.getSimData();
  SimInput simInput = simConnectInterface.getSimInput();

  // detect pause
  bool isInPause = false;
  if ((simData.simulationTime == previousSimulationTime) || (simData.simulationTime < 0.2)) {
    isInPause = true;
  }
  previousSimulationTime = simData.simulationTime;

  // fill time into model
  model.FlyByWire_U.in.time.dt = sampleTime;

  // fill data into model
  model.FlyByWire_U.in.data.nz_g = simData.nz_g;
  model.FlyByWire_U.in.data.Theta_deg = simData.Theta_deg;
  model.FlyByWire_U.in.data.Phi_deg = simData.Phi_deg;
  model.FlyByWire_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
  model.FlyByWire_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
  model.FlyByWire_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
  model.FlyByWire_U.in.data.q_dot_rad_s2 = simData.bodyRotationAcceleration.x;
  model.FlyByWire_U.in.data.r_dot_rad_s2 = simData.bodyRotationAcceleration.y;
  model.FlyByWire_U.in.data.p_dot_rad_s2 = simData.bodyRotationAcceleration.z;
  model.FlyByWire_U.in.data.eta_pos = simData.eta_pos;
  model.FlyByWire_U.in.data.eta_trim_deg = simData.eta_trim_deg;
  model.FlyByWire_U.in.data.xi_pos = simData.xi_pos;
  model.FlyByWire_U.in.data.zeta_pos = simData.zeta_pos;
  model.FlyByWire_U.in.data.zeta_trim_pos = simData.zeta_trim_pos;
  model.FlyByWire_U.in.data.alpha_deg = simData.alpha_deg;
  model.FlyByWire_U.in.data.beta_deg = simData.beta_deg;
  model.FlyByWire_U.in.data.beta_dot_deg_s = simData.beta_dot_deg_s;
  model.FlyByWire_U.in.data.V_ias_kn = simData.V_ias_kn;
  model.FlyByWire_U.in.data.V_tas_kn = simData.V_tas_kn;
  model.FlyByWire_U.in.data.V_mach = simData.V_mach;
  model.FlyByWire_U.in.data.H_ft = simData.H_ft;
  model.FlyByWire_U.in.data.H_ind_ft = simData.H_ind_ft;
  model.FlyByWire_U.in.data.H_radio_ft = simData.H_radio_ft;
  model.FlyByWire_U.in.data.CG_percent_MAC = simData.CG_percent_MAC;
  model.FlyByWire_U.in.data.gear_animation_pos_0 = simData.gear_animation_pos_0;
  model.FlyByWire_U.in.data.gear_animation_pos_1 = simData.gear_animation_pos_1;
  model.FlyByWire_U.in.data.gear_animation_pos_2 = simData.gear_animation_pos_2;
  model.FlyByWire_U.in.data.flaps_handle_index = simData.flaps_handle_index;
  model.FlyByWire_U.in.data.autopilot_master_on = simData.autopilot_master_on;
  model.FlyByWire_U.in.data.slew_on = simData.slew_on;
  model.FlyByWire_U.in.data.pause_on = isInPause;

  // process the sidestick handling
  // use the values read from input as sidestick left
  double sideStickLeftPositionX = -1.0 * simInput.inputs[1];
  double sideStickLeftPositionY = -1.0 * simInput.inputs[0];
  // read the values from sidestick right
  double sideStickRightPositionX = get_named_variable_value(idSideStickRightPositionX);
  double sideStickRightPositionY = get_named_variable_value(idSideStickRightPositionY);
  // add them together and clamp them
  double sideStickPositionX = sideStickLeftPositionX + sideStickRightPositionX;
  sideStickPositionX = min(1.0, sideStickPositionX);
  sideStickPositionX = max(-1.0, sideStickPositionX);
  double sideStickPositionY = sideStickLeftPositionY + sideStickRightPositionY;
  sideStickPositionY = min(1.0, sideStickPositionY);
  sideStickPositionY = max(-1.0, sideStickPositionY);
  // write them as sidestick position
  set_named_variable_value(idSideStickLeftPositionX, sideStickLeftPositionX);
  set_named_variable_value(idSideStickLeftPositionY, sideStickLeftPositionY);
  set_named_variable_value(idSideStickPositionX, sideStickPositionX);
  set_named_variable_value(idSideStickPositionY, sideStickPositionY);

  // rudder handling
  double rudderPositionOverrideOn = get_named_variable_value(idRudderPositionOverrideOn);
  double rudderPosition = simInput.inputs[2];
  if (rudderPositionOverrideOn == 0) {
    set_named_variable_value(idRudderPosition, rudderPosition);
  } else {
    rudderPosition = get_named_variable_value(idRudderPosition);
  }

  // fill inputs into model
  model.FlyByWire_U.in.input.delta_eta_pos = -1.0 * sideStickPositionY;
  model.FlyByWire_U.in.input.delta_xi_pos = -1.0 * sideStickPositionX;
  model.FlyByWire_U.in.input.delta_zeta_pos = rudderPosition;

  // success
  return true;
}

bool FlyByWireInterface::writeModelOuputDataToSim() {
  // when tracking mode is on do not write anything
  if (model.FlyByWire_Y.out.sim.data_computed.tracking_mode_on) {
    return true;
  }

  // object to write with trim
  SimOutput output = {model.FlyByWire_Y.out.sim.raw.output.eta_pos, model.FlyByWire_Y.out.sim.raw.output.xi_pos,
                      model.FlyByWire_Y.out.sim.raw.output.zeta_pos};

  // send data via sim connect
  if (!simConnectInterface.sendData(output)) {
    cout << "WASM: Write data failed!" << endl;
    return false;
  }

  if (model.FlyByWire_Y.out.sim.raw.output.eta_trim_deg_should_write) {
    // object to write without trim
    SimOutputEtaTrim output = {model.FlyByWire_Y.out.sim.raw.output.eta_trim_deg};

    // send data via sim connect
    if (!simConnectInterface.sendData(output)) {
      cout << "WASM: Write data failed!" << endl;
      return false;
    }
  }

  return true;
}

void FlyByWireInterface::initializeThrottles() {
  // read configuration
  INIReader configuration(THROTTLE_CONFIGURATION_FILEPATH);
  if (configuration.ParseError() < 0) {
    // file does not exist yet -> store the default configuration in a file
    ofstream configFile;
    configFile.open(THROTTLE_CONFIGURATION_FILEPATH);
    configFile << "[Throttle]" << endl;
    configFile << "Log = true" << endl;
    configFile << "Enabled = true" << endl;
    configFile << "ReverseOnAxis = false" << endl;
    configFile << "DetentReverseFull = -1.00" << endl;
    configFile << "DetentIdle = -1.00" << endl;
    configFile << "DetentClimb = 0.89" << endl;
    configFile << "DetentFlexMct = 0.95" << endl;
    configFile << "DetentTakeOffGoAround = 1.00" << endl;
    configFile.close();
  }

  // read basic configuration
  isThrottleLoggingEnabled = configuration.GetBoolean("Throttle", "Log", true);
  isThrottleHandlingEnabled = configuration.GetBoolean("Throttle", "Enabled", true);
  useReverseOnAxis = configuration.GetBoolean("Throttle", "ReverseOnAxis", false);
  // read mapping configuration
  vector<pair<double, double>> mappingTable;
  if (useReverseOnAxis) {
    mappingTable.emplace_back(configuration.GetReal("Throttle", "DetendReverseFull", -1.00), -20.00);
  }
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentIdle", useReverseOnAxis ? 0.00 : -1.00), 0.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentClimb", 0.89), 89.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentFlexMct", 0.95), 95.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentTakeOffGoAround", 1.00), 100.00);

  // remember idle throttle setting
  if (useReverseOnAxis) {
    idleThrottleInput = mappingTable[1].first;
  } else {
    idleThrottleInput = mappingTable[0].first;
  }

  // print config
  cout << "WASM: Throttle Configuration : Log                   = " << isThrottleLoggingEnabled << endl;
  cout << "WASM: Throttle Configuration : Enabled               = " << isThrottleHandlingEnabled << endl;
  cout << "WASM: Throttle Configuration : ReverseOnAxis         = " << useReverseOnAxis << endl;
  int index = 0;
  if (useReverseOnAxis) {
    cout << "WASM: Throttle Configuration : DetentReverseFull     = " << mappingTable[index++].first << endl;
  }
  cout << "WASM: Throttle Configuration : DetentIdle            = " << mappingTable[index++].first << endl;
  cout << "WASM: Throttle Configuration : DetentClimb           = " << mappingTable[index++].first << endl;
  cout << "WASM: Throttle Configuration : DetentFlexMct         = " << mappingTable[index++].first << endl;
  cout << "WASM: Throttle Configuration : DetentTakeOffGoAround = " << mappingTable[index++].first << endl;

  // initialize lookup table
  throttleLookupTable.initialize(mappingTable, -20, 100);
}

bool FlyByWireInterface::processThrottles() {
  // get data from simconnect
  auto simInputThrottles = simConnectInterface.getSimInputThrottles();

  // process the data (lut)
  SimOutputThrottles simOutputThrottles = {throttleLookupTable.get(simInputThrottles.throttles[0]),
                                           throttleLookupTable.get(simInputThrottles.throttles[1])};

  // detect reverse situation
  if (!useReverseOnAxis && simConnectInterface.getIsReverseToggleActive(0)) {
    simOutputThrottles.throttleLeverPosition_1 = -10.0 * (simInputThrottles.throttles[0] + 1);
  }
  if (!useReverseOnAxis && simConnectInterface.getIsReverseToggleActive(1)) {
    simOutputThrottles.throttleLeverPosition_2 = -10.0 * (simInputThrottles.throttles[1] + 1);
  }

  // detect if override is active
  if (get_named_variable_value(idThrottlePositionOverrideOn) == 0) {
    set_named_variable_value(idThrottlePosition_1, simOutputThrottles.throttleLeverPosition_1);
    set_named_variable_value(idThrottlePosition_2, simOutputThrottles.throttleLeverPosition_2);
  } else {
    simOutputThrottles.throttleLeverPosition_1 = get_named_variable_value(idThrottlePosition_1);
    simOutputThrottles.throttleLeverPosition_2 = get_named_variable_value(idThrottlePosition_2);
  }

  // if enabled, print values
  if (isThrottleLoggingEnabled) {
    if (lastUseReverseOnAxis != useReverseOnAxis || lastThrottleInput_1 != simInputThrottles.throttles[0] ||
        lastThrottleInput_2 != simInputThrottles.throttles[1]) {
      // print values
      cout << fixed << setprecision(2) << "WASM";
      cout << " : Throttle 1: " << setw(5) << simInputThrottles.throttles[0];
      cout << " -> " << setw(6) << simOutputThrottles.throttleLeverPosition_1;
      cout << " ; Throttle 2: " << setw(5) << simInputThrottles.throttles[1];
      cout << " -> " << setw(6) << simOutputThrottles.throttleLeverPosition_2;
      cout << endl;

      // store values for next iteration
      lastUseReverseOnAxis = useReverseOnAxis;
      lastThrottleInput_1 = simInputThrottles.throttles[0];
      lastThrottleInput_2 = simInputThrottles.throttles[1];
    }
  }

  // write output to sim
  if (!simConnectInterface.sendData(simOutputThrottles)) {
    cout << "WASM: Write data failed!" << endl;
    return false;
  }

  // determine if autothrust armed event needs to be triggered
  if (simConnectInterface.getIsAutothrottlesArmed() && simOutputThrottles.throttleLeverPosition_1 < 1 &&
      simOutputThrottles.throttleLeverPosition_2 < 1) {
    if (!simConnectInterface.sendAutoThrustArmEvent()) {
      cout << "WASM: Write data failed!" << endl;
      return false;
    }
  }

  // success
  return true;
}
