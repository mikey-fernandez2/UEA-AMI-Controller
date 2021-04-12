// Print robot state - current joint positions, velocities, sensor readings
void printState(const hxRobotInfo *_robotInfo, const hxSensor *_sensor);

// Print robot info - number of joints, limits
void printRobotInfo(const hxRobotInfo *_robotInfo);

// Print next command to be sent
void printCommand(const hxRobotInfo *_robotInfo, const hxCommand *_cmd);

// Print EMG struct
void printEMGData(const struct EMGData *emg);

// Print EMG normalization factors
void printEMGNorms(float *norms);

// Print tracking data from Polhemus system
void printPolhemus(polhemus_pose_t *poses, int num_poses);