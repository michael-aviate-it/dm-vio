/**
 * This file is based on the file main_dso_pangolin.cpp of the project DSO
 * written by Jakob Engel. It has been heavily modified by Lukas von Stumberg
 * for the inclusion in DM-VIO (http://vision.in.tum.de/dm-vio).
 *
 * Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

// Main file for running live on a Luxonis camera, based on the main file of DSO.

#include <locale.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"

#include "util/Undistort.h"

#include "dso/util/globalCalib.h"
#include "dso/util/globalFuncs.h"
#include "dso/util/settings.h"
#include "util/TimeMeasurement.h"
#include <boost/thread.hpp>

#include "FullSystem/FullSystem.h"
#include "FullSystem/PixelSelector2.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "dso/util/NumType.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "live/FrameSkippingStrategy.h"
#include "live/Luxonis.h"
#include "util/MainSettings.h"

#include <boost/filesystem.hpp>

// If mainSettings.calib is set we use this instead of the factory calibration.
std::string calibSavePath = "./factoryCalibrationLuxonis.txt";

// Factory camchain will be saved here if set.
std::string camchainSavePath = "";
int start = 2;

using namespace dso;

dmvio::FrameContainer frameContainer;
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<Undistort> undistorter;
std::unique_ptr<dmvio::DatasetSaver> datasetSaver;
std::string saveDatasetPath = "";

// ************************************************************************************
void my_exit_handler(int s) {
	printf("Caught signal %d\n", s);
	exit(1);
}

// ************************************************************************************
void exitThread() {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	while (true)
		pause();
}

// ************************************************************************************
void run(IOWrap::PangolinDSOViewer *viewer, Undistort *undistorter) {
	std::cout << "run(IOWrap::PangolinDSOViewer *viewer)" << std::endl;

	bool linearizeOperation = false;
	auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);

	if (setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr) {
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}

	if (undistorter->photometricUndist != nullptr) {
		fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
	}

	fullSystem->setGammaFunction(0);

	if (viewer != 0) {
		fullSystem->outputWrapper.push_back(viewer);
	}

	dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);

	fullSystem->outputWrapper.push_back(&frameSkipping);

	int image_index = 0;
	int lastResetIndex = 0;

	std::cout << "while (true)" << std::endl;
	while (true) {
		if (start > 0 && image_index < start) {
			auto pair = frameContainer.getImageAndIMUData();

			++image_index;
			continue;
		}

		// std::cout << "getImageAndIMUData" << std::endl;
		auto pair = frameContainer.getImageAndIMUData(frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize()));

		fullSystem->addActiveFrame(pair.first.get(), image_index, &(pair.second), nullptr);


		// fix: commenting this out makes the init much faster!
		// if (fullSystem->initFailed || setting_fullResetRequested) {
		// 	std::cout << "initFailed" << std::endl;
		// 	if (image_index - lastResetIndex < 250 || setting_fullResetRequested) {
		// 		printf("RESETTING!\n");
		// 		std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
		// 		fullSystem.reset();
		// 		for (IOWrap::Output3DWrapper *output_wrapper : wraps)
		// 			output_wrapper->reset();

		// 		fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
		// 		if (undistorter->photometricUndist != nullptr) {
		// 			fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		// 		}
		// 		fullSystem->outputWrapper = wraps;

		// 		setting_fullResetRequested = false;
		// 		lastResetIndex = image_index;
		// 	}
		// }

		if (viewer != nullptr && viewer->shouldQuit()) {
			std::cout << "User closed window -> Quit!" << std::endl;
			break;
		}

		if (fullSystem->isLost) {
			printf("LOST!!\n");
			break;
		}

		++image_index;
	}

	fullSystem->blockUntilMappingIsFinished();

	fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);

	dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

	for (IOWrap::Output3DWrapper *output_wrapper : fullSystem->outputWrapper) {
		output_wrapper->join();
	}

	printf("DELETE FULLSYSTEM!\n");
	fullSystem.reset();

	if (datasetSaver)
		datasetSaver->end();

	printf("EXIT NOW!\n");
}

// ************************************************************************************
int main(int argc, char **argv) {
	setlocale(LC_ALL, "C");

#ifdef DEBUG
	std::cout << "DEBUG MODE!" << std::endl;
#endif

	auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

	// Create Settings files.
	imuSettings.registerArgs(*settingsUtil);
	imuCalibration.registerArgs(*settingsUtil);
	mainSettings.registerArgs(*settingsUtil);
	frameSkippingSettings.registerArgs(*settingsUtil);

	settingsUtil->registerArg("start", start);
	settingsUtil->registerArg("calibSavePath", calibSavePath);
	settingsUtil->registerArg("camchainSavePath", camchainSavePath);
	settingsUtil->registerArg("saveDatasetPath", saveDatasetPath);

	auto normalizeCamSize = std::make_shared<double>(0.0);
	settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);

	// This call will parse all commandline arguments and potentially also read a
	// settings yaml file if passed.
	mainSettings.parseArguments(argc, argv, *settingsUtil);

	// Print settings to commandline and file.
	std::cout << "Settings:\n";
	settingsUtil->printAllSettings(std::cout);
	{
		std::ofstream settingsStream;
		settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
		settingsUtil->printAllSettings(settingsStream);
	}

	// hook crtl+C.
	boost::thread exThread = boost::thread(exitThread);

	if (saveDatasetPath != "") {
		try {
			datasetSaver = std::make_unique<dmvio::DatasetSaver>(saveDatasetPath);
		} catch (const boost::filesystem::filesystem_error &err) {
			std::cout << "ERROR: Cannot save dataset: " << err.what() << std::endl;
		}
	}

	std::cout << "Saving camera calibration to " << calibSavePath << "\n";
	dmvio::Luxonis luxonis(frameContainer, calibSavePath, datasetSaver.get());

	luxonis.start();

	std::cout << "mainSettings.calib" << std::endl;
	std::string usedCalib = calibSavePath;
	if (mainSettings.calib != "") {
		usedCalib = mainSettings.calib;
		std::cout << "Using custom camera calibration (instead of factory calibration): " << mainSettings.calib << "\n";
	}
	
	std::cout << "camchainSavePath" << std::endl;
	if (camchainSavePath != "") {
		std::cout << "Saving T_cam_imu to: " << camchainSavePath << std::endl;
		luxonis.imuCalibration->saveToFile(camchainSavePath);
	}

	std::cout << "undistorter" << std::endl;
	std::unique_ptr<Undistort> undistorter(Undistort::getUndistorterForFile(usedCalib, mainSettings.gammaCalib, mainSettings.vignette));
	luxonis.setUndistorter(undistorter.get());

	std::cout << "setGlobalCalib" << std::endl;
	setGlobalCalib(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1],
		undistorter->getK().cast<float>()
	);

	// std::cout << "mainSettings.imuCalibFile" << std::endl;
	// if (mainSettings.imuCalibFile != "") {
		imuCalibration.loadFromFile(mainSettings.imuCalibFile);
	// } else {
	// 	std::cout << "Using factory IMU calibration!" << std::endl;
	// 	imuCalibration = *(luxonis.imuCalibration);
	// }

	// if (!disableAllDisplay) {
		std::cout << "PangolinDSOViewer" << std::endl;
		IOWrap::PangolinDSOViewer *viewer = new IOWrap::PangolinDSOViewer(wG[0], hG[0], false, settingsUtil, normalizeCamSize);

		std::cout << "runThread" << std::endl;
		boost::thread runThread = boost::thread(boost::bind(run, viewer, undistorter.get()));

		viewer->run();

		delete viewer;

		// Make sure that the destructor of FullSystem, etc. finishes, so all log
		// files are properly flushed.
		runThread.join();
	// } else {
	// 	run(0, undistorter.get());
	// }

	return 0;
}
