// Functions to log data from HAPTIX Gazebo and commands
//
// Mikey Fernandes 04/23/2021

#include "../include/handsim/logging.h"

int startLogging(char *logPath, bool usingEMG, bool usingPolhemus, hxRobotInfo *robotInfo, struct EMGData *emg)
{
    FILE *log;
    log = fopen(logPath, "w");

    if (log == NULL)
    {
        return 0;
    }

    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    fprintf(log, "This log file was generated automatically starting at %s\n", asctime(timeinfo));
    fprintf(log, "Settings for this run:\n\tusingEMG? %s\n\tusingPolhemus? %s\n", usingEMG ? "true" : "false", usingPolhemus ? "true" : "false");

    fprintf(log, "\nRobot information:\n");
    fprintf(log, "\tNum motors: %d\n", robotInfo->motor_count);
    fprintf(log, "\tNum joints: %d\n", robotInfo->joint_count);
    fprintf(log, "\tNum contact sensors: %d\n", robotInfo->contact_sensor_count);
    fprintf(log, "\tNum IMUs: %d\n", robotInfo->imu_count);
    fprintf(log, "\tUpdate rate: %06.2f Hz\n", robotInfo->update_rate);
    fprintf(log, "\tActuated joint limits: \n");

    // Print joint limits.
    int i;
    for (i = 0; i < robotInfo->motor_count; ++i)
    {
        fprintf(log, "\t\tJoint associated to motor %d:\n", i);
        fprintf(log, "\t\t\tMin: %06.3f rads\n", robotInfo->joint_limit[i][0]);
        fprintf(log, "\t\t\tMax: %06.3f rads\n", robotInfo->joint_limit[i][1]);
    }

    if (usingEMG)
    {
        fprintf(log, "\nActivation Time Constant: %05.3f sec\nDeactivation Time Constant: %05.3f sec\n", emg->tauA, emg->tauD);
        fprintf(log, "\nEMG Bounds and Max/Min Differences:\n");

        fprintf(log, "Electrode: %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d\n",
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
        fprintf(log, "\tMaxes: %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f\n",
         emg->bounds[0], emg->bounds[1], emg->bounds[2], emg->bounds[3], emg->bounds[4], emg->bounds[5], emg->bounds[6], emg->bounds[7], emg->bounds[8], emg->bounds[9], emg->bounds[10], emg->bounds[11], emg->bounds[12], emg->bounds[13], emg->bounds[14], emg->bounds[15]);
        fprintf(log, "\tMins:  %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f\n",
         emg->bounds[16], emg->bounds[17], emg->bounds[18], emg->bounds[19], emg->bounds[20], emg->bounds[21], emg->bounds[22], emg->bounds[23], emg->bounds[24], emg->bounds[25], emg->bounds[26], emg->bounds[27], emg->bounds[28], emg->bounds[29], emg->bounds[30], emg->bounds[31]);

        fprintf(log, "Pair:          %3d-%2d, %3d-%2d, %3d-%2d, %3d-%2d, %3d-%2d, %3d-%2d, %3d-%2d, %3d-%2d\n",
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
        fprintf(log, "\tMax Delta: %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f\n",
         emg->deltas[0], emg->deltas[1], emg->deltas[2], emg->deltas[3], emg->deltas[4], emg->deltas[5], emg->deltas[6], emg->deltas[7]);
        fprintf(log, "\tMin Delta: %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f, %06.3f\n",
         emg->deltas[8], emg->deltas[9], emg->deltas[10], emg->deltas[11], emg->deltas[12], emg->deltas[13], emg->deltas[14], emg->deltas[15]);
    }

    fprintf(log, "\n--------------------------------------------------\n");

    fclose(log);

    return 1;
}

int addLog(char *logPath, bool usingEMG, bool usingPolhemus, double runTime, hxRobotInfo *robotInfo, hxCommand *cmd, hxSensor *sensor, polhemus_pose_t *poses, int num_poses, struct EMGData *emg)
{
    FILE *log;
    log = fopen(logPath, "a");

    if (log == NULL)
    {
        return 0;
    }

    int i;

    // Times
    fprintf(log, "\nSimulation Time: %d.%09d sec\n", sensor->time_stamp.sec, sensor->time_stamp.nsec);
    fprintf(log, "Running Time: %f sec\n", runTime);

    // Command
    fprintf(log, "\nCommand:\n");
    fprintf(log, "\tMotor:\n");
    for (i = 0; i < robotInfo->motor_count; ++i)
    {
    fprintf(log, "\t\tMotor %d\n", i);
    fprintf(log, "\t\t\tref_pos: %06.2f rads\n", cmd->ref_pos[i]);
    }

    // Sensors
    fprintf(log, "\nSensors:\n");
    fprintf(log, "\tMotors:\n");
    for (i = 0; i < robotInfo->motor_count; ++i)
    {
    fprintf(log, "\t\tMotor %d\n", i);
    fprintf(log, "\t\t\tPosition: %06.2f rads\n", sensor->motor_pos[i]);
    }
    fprintf(log, "\tJoints:\n");
    for (i = 0; i < robotInfo->joint_count; ++i)
    {
    fprintf(log, "\t\tJoint %d\n", i);
    fprintf(log, "\t\t\tPosition: %07.4f rads\n", sensor->joint_pos[i]);
    }
    fprintf(log, "\n");

    // EMG
    if (usingEMG)
    {
        fprintf(log, "EMG:\n");
        fprintf(log, "\tReceiving Frequency: %06.3f\n", emg->samplingFreq);
        fprintf(log, "\tTrigger: %d\n", emg->trigger);
        fprintf(log, "\tSwitch1: %d\n", emg->switch1);
        fprintf(log, "\tSwitch2: %d\n", emg->switch2);

        fprintf(log, "\tRaw EMG:\n");
        fprintf(log, "\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
            emg->rawEMG[0], emg->rawEMG[4], emg->rawEMG[8],  emg->rawEMG[12],
            emg->rawEMG[1], emg->rawEMG[5], emg->rawEMG[9],  emg->rawEMG[13],
            emg->rawEMG[2], emg->rawEMG[6], emg->rawEMG[10], emg->rawEMG[14],
            emg->rawEMG[3], emg->rawEMG[7], emg->rawEMG[11], emg->rawEMG[15]);

        fprintf(log, "\tNormed EMG:\n");
        fprintf(log, "\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
            emg->normedEMG[0], emg->normedEMG[4], emg->normedEMG[8],  emg->normedEMG[12],
            emg->normedEMG[1], emg->normedEMG[5], emg->normedEMG[9],  emg->normedEMG[13],
            emg->normedEMG[2], emg->normedEMG[6], emg->normedEMG[10], emg->normedEMG[14],
            emg->normedEMG[3], emg->normedEMG[7], emg->normedEMG[11], emg->normedEMG[15]);

        fprintf(log, "\tMuscle Activation:\n");
        fprintf(log, "\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n\t\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n",
            emg->muscleAct[0], emg->muscleAct[4], emg->muscleAct[8],  emg->muscleAct[12],
            emg->muscleAct[1], emg->muscleAct[5], emg->muscleAct[9],  emg->muscleAct[13],
            emg->muscleAct[2], emg->muscleAct[6], emg->muscleAct[10], emg->muscleAct[14],
            emg->muscleAct[3], emg->muscleAct[7], emg->muscleAct[11], emg->muscleAct[15]);

        if (usingPolhemus)
        {
            fprintf(log, "\n");
        }

    }

    // Polhemus
    if (usingPolhemus)
    {
        fprintf(log, "Polhemus:\n");

        for(i = 0; i < num_poses; i++)
        {
            fprintf(log, "\tStation %d: %lf %lf %lf %lf %lf %lf\n", poses[i].station_id,  poses[i].x, poses[i].y, poses[i].z, poses[i].roll, poses[i].pitch, poses[i].yaw);
        }

    }

    fprintf(log, "\n--------------------------------------------------\n");

    fclose(log);

    return 1;
}

int endLog(char *logPath)
{
    FILE *log;
    log = fopen(logPath, "a");

    if (log == NULL)
    {
        return 0;
    }

    time_t rawtime;
    struct tm * timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    fprintf(log, "\nEnding time: %s", asctime(timeinfo));

    fclose(log);

    return 1;
}