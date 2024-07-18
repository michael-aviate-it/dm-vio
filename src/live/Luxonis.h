/**
* This file is part of DM-VIO.
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DMVIO_LUXONIS_H
#define DMVIO_LUXONIS_H

#include <iostream>
#include "depthai/depthai.hpp"

#include "IMU/IMUSettings.h"
#include "live/IMUInterpolator.h"
#include "FrameContainer.h"
#include "util/Undistort.h"
#include "DatasetSaver.h"

namespace dmvio {
    class Luxonis {
        public:
            Luxonis(FrameContainer& frameContainer, std::string cameraCalibSavePath, DatasetSaver* datasetSaver);

            void start();

            void setUndistorter(dso::Undistort* undistort);

            std::unique_ptr<IMUCalibration> imuCalibration;

            void callback_camera(std::vector<std::shared_ptr<dai::ImgFrame>> img_frames);
            void callback_imu(std::shared_ptr<dai::IMUData> imu_data);

        private:
            dai::Pipeline pipeline;

            std::chrono::time_point<std::chrono::steady_clock> start_time_camera;
            uint16_t fps_counter_camera = 0;

            std::chrono::time_point<std::chrono::steady_clock> start_time_imu;
            uint16_t fps_counter_imu = 0;

            std::string cameraCalibSavePath;

            std::atomic<bool> calibrationRead{false};

            IMUInterpolator imu_interpolator;
            dso::Undistort* undistorter = nullptr;
            double lastImgTimestamp = -1.0;

            DatasetSaver* saver;

            void readCalibration();
    }; // class Luxonis

} // namespace dmvio

#endif // DMVIO_LUXONIS_H
