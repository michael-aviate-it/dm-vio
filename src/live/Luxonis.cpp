#include "IMU/IMUTypes.h"
#include "Luxonis.h"
#include "sophus/se3.hpp"
#include "util/MinimalImage.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace dai;
using namespace dmvio;
using namespace dso;

using Callback = std::function<void(std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::IMUData>)>;

void fnc_thread_luxonis(Callback callback, std::shared_ptr<dai::Pipeline> pipeline) {
	dai::Device device(*pipeline, dai::UsbSpeed::SUPER);

	// std::cout << "Usb speed: " << device.getUsbSpeed() << std::endl;

	// std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << std::endl;

	// if(device.getBootloaderVersion()) {
	// 	std::cout << "Bootloader version: " << device.getBootloaderVersion()->toString() << std::endl;
	// }

	// std::cout << "Connected cameras: " << device.getConnectedCameraFeatures() << std::endl;

	// auto imuType = device.getConnectedIMU();
	// auto imuFirmwareVersion = device.getIMUFirmwareVersion();
	// std::cout << "IMU type: " << imuType << ", firmware version: " << imuFirmwareVersion << std::endl;

	// std::cout << "ouput_queue_video" << std::endl;
	auto queue_group = device.getOutputQueue("x_link_out", 4, false);

	// std::cout << "startTime" << std::endl;
	auto startTime = std::chrono::steady_clock::now();
	int fps_counter_video = 0;
	float fps_video = 0;

	int fps_counter_imu = 0;
	float fps_imu = 0;

	// std::cout << "entering while" << std::endl;
	while(true) {
		auto message_group = queue_group->get<dai::MessageGroup>();
		auto img_frame = message_group->get<dai::ImgFrame>("video");
		auto imu_data = message_group->get<dai::IMUData>("imu");

		auto currentTime = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - startTime);
		if(elapsed > std::chrono::seconds(1)) {
			fps_video = fps_counter_video / elapsed.count();
			std::cout << "FPS Video: " << fps_video << std::endl;
			fps_counter_video = 0;

			fps_imu = fps_counter_imu / elapsed.count();
			std::cout << "FPS IMU: " << fps_imu << std::endl;
			fps_counter_imu = 0;
			startTime = currentTime;
		}

		if (img_frame) {
			fps_counter_video++;

		// 	cv::imshow("video", img_frame->getCvFrame());
		// 	int key = cv::waitKey(1);
		// 	if(key == 'q' || key == 'Q') {
		// 		break;
		// 	}
		}

		if (imu_data) {
			fps_counter_imu++;
		}

		callback(img_frame, imu_data);
	}
}

dmvio::Luxonis::Luxonis(	FrameContainer &frameContainer,
							std::string cameraCalibSavePath,
							DatasetSaver *datasetSaver) : 	imu_interpolator(frameContainer, datasetSaver),
															cameraCalibSavePath(cameraCalibSavePath),
															saver(datasetSaver) {
}

void dmvio::Luxonis::start() {
	// std::cout << "dmvio::Luxonis::start()" << std::endl;
	readCalibration();

	auto pipeline = std::make_shared<dai::Pipeline>();

	auto camera = pipeline->create<dai::node::ColorCamera>();
	// auto camera = pipeline->create<dai::node::Camera>();
	// camera->setBoardSocket(dai::CameraBoardSocket::CAM_A);
	camera->setBoardSocket(dai::CameraBoardSocket::CAM_B);
	camera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
	camera->setVideoSize(1280, 720);

	// auto camera = pipeline->create<dai::node::MonoCamera>();
	// camera->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    // camera->setCamera("left");


	auto imu = pipeline->create<dai::node::IMU>();
	imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 60);
	imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 60);
	// imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 30);
	imu->setBatchReportThreshold(1);
	imu->setMaxBatchReports(10);

	auto sync = pipeline->create<dai::node::Sync>();
	sync->setSyncThreshold(std::chrono::milliseconds(10));
	sync->setSyncAttempts(-1);	

	// std::cout << "link" << std::endl;
	// ColorCamera & Camera
	camera->video.link(sync->inputs["video"]);
	// MonoCamera
	// camera->out.link(sync->inputs["video"]);
	imu->out.link(sync->inputs["imu"]);

	// std::cout << "x_link_out" << std::endl;
	auto x_link_out = pipeline->create<dai::node::XLinkOut>();
	x_link_out->setStreamName("x_link_out");
	// x_link_out->input.setBlocking(false);
	// x_link_out->input.setQueueSize(1);
	sync->out.link(x_link_out->input);

	Callback callback = [this](std::shared_ptr<dai::ImgFrame> img_frame, std::shared_ptr<dai::IMUData> imu_data) {
		// std::cout << "callback_image" << std::endl;
		cv::Mat mat = img_frame->getCvFrame();
		cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);

		// cv::imshow("mat", mat);
		// int key = cv::waitKey(1);
		// if(key == 'q' || key == 'Q') {
		// 	std::cout << "nope" << std::endl;
		// }

		// int type = mat.type();

		// std::string rtype;
		// int depth = type & CV_MAT_DEPTH_MASK;
		// switch (depth) {
		// 	case CV_8U:  rtype = "CV_8U"; break;
		// 	case CV_8S:  rtype = "CV_8S"; break;
		// 	case CV_16U: rtype = "CV_16U"; break;
		// 	case CV_16S: rtype = "CV_16S"; break;
		// 	case CV_32S: rtype = "CV_32S"; break;
		// 	case CV_32F: rtype = "CV_32F"; break;
		// 	case CV_64F: rtype = "CV_64F"; break;
		// 	default:     rtype = "User"; break;
		// }

		// int channels = 1 + (type >> CV_CN_SHIFT);
		// std::cout << "CV_MAT_DEPTH_MASK: " << rtype << std::endl;
		// std::cout << "Channels: " << channels << std::endl;
		// std::cout << "Total elements: " << mat.total() << std::endl;
		// std::cout << "Element size (in bytes): " << mat.elemSize() << std::endl;
		// Depth: CV_8U
		// Channels: 3
		// Total elements: 921600
		// Element size (in bytes): 3		

		auto timestamp = img_frame->getTimestamp();
		auto exposure_us = img_frame->getExposureTime();
		auto exposure_ms = std::chrono::duration_cast<std::chrono::milliseconds>(exposure_us).count();

		// std::cout << "mat.type(): " << mat.type() << std::endl;
		// std::cout << "CV_8U: " << CV_8U << std::endl;
		// assert(mat.type() == CV_8U);
		// std::cout << "MinimalImageB" << std::endl;
		// MinimalImageB img((int)mat.cols, (int)mat.rows);
		// memcpy(img.data, mat.data, mat.rows * mat.cols);
		auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
		memcpy(img->data, mat.data, mat.rows * mat.cols);

		// std::cout << "duration_since_epoch" << std::endl;
		auto duration_since_epoch = timestamp.time_since_epoch();
		auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch).count();
		double timestamp_dbl = static_cast<double>(seconds_since_epoch);

		// std::cout << "ImageAndExposure" << std::endl;
		// std::unique_ptr<ImageAndExposure> image_and_exposure(undistorter->undistort<unsigned char>(&img, 1.0, timestamp_dbl, 1.0f));
		std::unique_ptr<ImageAndExposure> image_and_exposure(undistorter->undistort<unsigned char>(img.get(), 1.0, timestamp_dbl));
		img.reset();

		// std::cout << "addImage" << std::endl;
		imu_interpolator.addImage(std::move(image_and_exposure), timestamp_dbl);


		std::vector<float> accData;
		std::vector<float> gyrData;
		auto imu_packets = imu_data->packets;
		// int counter_imu_packets = 0;
		for(auto& imu_packet : imu_packets) {
			// counter_imu_packets++;
			auto& acc = imu_packet.acceleroMeter;
			accData.push_back(acc.x);
			accData.push_back(acc.y);
			accData.push_back(acc.z);

			auto& gyro = imu_packet.gyroscope;
			gyrData.push_back(gyro.x);
			gyrData.push_back(gyro.y);
			gyrData.push_back(gyro.z);			
		}

		imu_interpolator.addAccData(accData, timestamp_dbl);
		imu_interpolator.addGyrData(gyrData, timestamp_dbl);

		// std::cout << "counter_imu_packets: " << counter_imu_packets << std::endl;
	};

	// std::thread thread_luxonis(fnc_thread_luxonis, callback_image, callback_imu, pipeline);
	std::thread thread_luxonis(fnc_thread_luxonis, callback, pipeline);
	thread_luxonis.detach();
}

void dmvio::Luxonis::readCalibration() {
	if (calibrationRead)
		return;

	calibrationRead = true;
}

void Luxonis::setUndistorter(Undistort *undistort) {
	std::cout << "setUndistorter" << std::endl;
	this->undistorter = undistort;
}
