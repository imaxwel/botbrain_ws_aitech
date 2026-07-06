export const SCENARIOS = {
  default: {
    name: 'default',
    initialMode: 'damp',
    initialEmergencyStop: false,
    batteryPercentage: 0.82,
    batteryDrainPerSecond: 0.000015,
    imuTemperatureC: 38.5,
    networkDropEveryMs: 0,
    diagnosticLevel: 0,
    diagnosticMessage: 'Mock G1 nominal',
  },
  low_battery: {
    name: 'low_battery',
    initialMode: 'damp',
    initialEmergencyStop: false,
    batteryPercentage: 0.18,
    batteryDrainPerSecond: 0.00003,
    imuTemperatureC: 39.0,
    networkDropEveryMs: 0,
    diagnosticLevel: 1,
    diagnosticMessage: 'Mock battery below warning threshold',
  },
  emergency: {
    name: 'emergency',
    initialMode: 'damp',
    initialEmergencyStop: true,
    batteryPercentage: 0.76,
    batteryDrainPerSecond: 0.000012,
    imuTemperatureC: 40.0,
    networkDropEveryMs: 0,
    diagnosticLevel: 2,
    diagnosticMessage: 'Mock emergency stop active',
  },
  unstable_network: {
    name: 'unstable_network',
    initialMode: 'damp',
    initialEmergencyStop: false,
    batteryPercentage: 0.67,
    batteryDrainPerSecond: 0.00002,
    imuTemperatureC: 41.5,
    networkDropEveryMs: 45000,
    diagnosticLevel: 1,
    diagnosticMessage: 'Mock network jitter scenario',
  },
};

export function getScenarioConfig(name = 'default') {
  return SCENARIOS[name] ?? SCENARIOS.default;
}

export function listScenarioNames() {
  return Object.keys(SCENARIOS);
}
