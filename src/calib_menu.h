#pragma once
#include <Arduino.h>

bool checkForQuit();
extern int waitForUser();
char waitForChoice(const char* options);
void changeLogicParameters();
void changeLightParameters();
void saveLightCalibration();
void loadLightCalibration();
void saveLogicParameters();
void loadLogicParameters();
void clearPreferences();
void saveCalibrationData(int concentrationIdx, float resultsFlow[4], float resultsStationary[4], float avgsFlow[4], float avgsStationary[4], int nMeas, const String& sampleName);
void printCalibrationCSV();
void csvMenu();
