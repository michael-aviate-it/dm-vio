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

// ************************************************************************************
void fnc_thread_luxonis(Luxonis* luxonis) {
	// std::cout << "fnc_thread_luxonis" << std::endl;
	dai::Device device;

	// std::cout << "Usb speed: " << device.getUsbSpeed() << std::endl;
	// std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << std::endl;
	// if(device.getBootloaderVersion()) {
	// 	std::cout << "Bootloader version: " << device.getBootloaderVersion()->toString() << std::endl;
	// }
	// std::cout << "Connected cameras: " << device.getConnectedCameraFeatures() << std::endl;
	// auto imuType = device.getConnectedIMU();
	// auto imuFirmwareVersion = device.getIMUFirmwareVersion();
	// std::cout << "IMU type: " << imuType << ", firmware version: " << imuFirmwareVersion << std::endl;
    // if(imuType != "BNO086") {
    //     std::cout << "Rotation vector output is supported only by BNO086!" << std::endl;
    //     return;
    // }	


    dai::Pipeline pipeline;

	// std::cout << "camera" << std::endl;
	// MONO
	// auto camera = pipeline.create<dai::node::MonoCamera>();
    // camera->setCamera("left");
	// camera->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

	// // std::cout << "x_link_out_camera" << std::endl;
	// auto x_link_out_camera = pipeline.create<dai::node::XLinkOut>();
	// x_link_out_camera->setStreamName("x_link_out_camera");
	// camera->out.link(x_link_out_camera->input);

	// COLOR
	auto camera = pipeline.create<dai::node::ColorCamera>();
	// auto camera = pipeline.create<dai::node::Camera>();

	// CAM_A = RGB / Center
	// CAM_B = LEFT
	// CAM_C = RIGHT
	camera->setBoardSocket(dai::CameraBoardSocket::CAM_A);
	// camera->setBoardSocket(dai::CameraBoardSocket::CAM_B);
	camera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
	camera->setVideoSize(1280, 720);

	// std::cout << "x_link_out_camera" << std::endl;
	auto x_link_out_camera = pipeline.create<dai::node::XLinkOut>();
	x_link_out_camera->setStreamName("x_link_out_camera");
	camera->video.link(x_link_out_camera->input);

    auto imu = pipeline.create<dai::node::IMU>();
	imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
	imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
	imu->setBatchReportThreshold(5);
	imu->setMaxBatchReports(20);

	// std::cout << "x_link_out_imu" << std::endl;
    auto x_link_out_imu = pipeline.create<dai::node::XLinkOut>();
	x_link_out_imu->setStreamName("x_link_out_imu");
	imu->out.link(x_link_out_imu->input);

	device.startPipeline(pipeline);

	auto output_queue_camera = device.getOutputQueue("x_link_out_camera", 20, false);
	auto output_queue_imu = device.getOutputQueue("x_link_out_imu", 200, false);

	// std::cout << "entering while" << std::endl;
	while(true) {
		auto image_frames = output_queue_camera->tryGetAll<dai::ImgFrame>();
		luxonis->callback_camera(image_frames);

		try {
			auto imu_data = output_queue_imu->get<dai::IMUData>();
			luxonis->callback_imu(imu_data);
		} catch(const std::exception &ex) {
			std::cout << "ERROR: Failed to get IMU Data" << std::endl;
			std::cout << "what(): " << ex.what() << std::endl;
		}
	}
}

// ************************************************************************************
dmvio::Luxonis::Luxonis(	FrameContainer &frameContainer,
							std::string cameraCalibSavePath,
							DatasetSaver *datasetSaver) : 	imu_interpolator(frameContainer, datasetSaver),
															cameraCalibSavePath(cameraCalibSavePath),
															saver(datasetSaver),
															start_time_camera(std::chrono::steady_clock::now()),
															start_time_imu(std::chrono::steady_clock::now()) {
}

// ************************************************************************************
void dmvio::Luxonis::callback_camera(std::vector<std::shared_ptr<dai::ImgFrame>> image_frames) {
	// std::cout << "callback_camera" << std::endl;

	auto current_time = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(current_time - start_time_camera);
	if(elapsed > std::chrono::seconds(1)) {
		float fps_camera = fps_counter_camera / elapsed.count();
		std::cout << "FPS Camera: " << fps_camera << std::endl;
		fps_counter_camera = 0;

		start_time_camera = current_time;
	} // if(elapsed > std::chrono::seconds(1)) {

	uint8_t size_image_frames = image_frames.size();
	if(size_image_frames > 0) {
		fps_counter_camera += size_image_frames;

		for(const auto& image_frame : image_frames) {
			cv::Mat cv_mat = image_frame->getCvFrame();
			cv::cvtColor(cv_mat, cv_mat, cv::COLOR_BGR2GRAY);

			// cv::imshow("Image Frame", cv_mat);
			// int key = cv::waitKey(1);
			// if(key == 'q' || key == 'Q') {
			// 	std::cout << "nope" << std::endl;
			// }

			auto minimal_image_b = std::make_unique<dso::MinimalImageB>(cv_mat.cols, cv_mat.rows);
			memcpy(minimal_image_b->data, cv_mat.data, cv_mat.rows * cv_mat.cols);

			auto timestamp_img = image_frame->getTimestamp();
			auto time_since_epoch_img = timestamp_img.time_since_epoch();
			auto duration_cast_img = std::chrono::duration_cast<std::chrono::duration<double>>(time_since_epoch_img);
			double timestamp_dbl_img = duration_cast_img.count();
			// std::cout << "timestamp_dbl_img: " << timestamp_dbl_img << std::endl;

			// std::cout << "ImageAndExposure" << std::endl;
			std::unique_ptr<ImageAndExposure> image_and_exposure(undistorter->undistort<unsigned char>(minimal_image_b.get(), 1.0, timestamp_dbl_img));
			minimal_image_b.reset();

			// std::cout << "addImage" << std::endl;
			imu_interpolator.addImage(std::move(image_and_exposure), timestamp_dbl_img);
		}
	}
} // void dmvio::Luxonis::callback_camera(std::vector<std::shared_ptr<dai::ImgFrame>> image_frames) {

// ************************************************************************************
void dmvio::Luxonis::callback_imu(std::shared_ptr<dai::IMUData> imu_data) {
	// std::cout << "callback_imu" << std::endl;

	auto current_time = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(current_time - start_time_imu);
	if(elapsed > std::chrono::seconds(1)) {
		float fps_imu = fps_counter_imu / elapsed.count();
		std::cout << "FPS IMU: " << fps_imu << std::endl;
		fps_counter_imu = 0;

		start_time_imu = current_time;
	} // if(elapsed > std::chrono::seconds(1)) {

	uint8_t size_imu_data_packets = imu_data->packets.size();
	if(size_imu_data_packets > 0) {
		fps_counter_imu += size_imu_data_packets;

		for(auto& imu_packet : imu_data->packets) {
			// todo: check if values are there and valid!
			auto& accelerometer = imu_packet.acceleroMeter;

			auto timestamp_acc = accelerometer.getTimestamp();
			auto time_since_epoch_acc = timestamp_acc.time_since_epoch();
			auto duration_cast_acc = std::chrono::duration_cast<std::chrono::duration<double>>(time_since_epoch_acc);
			double timestamp_dbl_acc = duration_cast_acc.count();
			// std::cout << "timestamp_dbl_acc: " << timestamp_dbl_acc << std::endl;

			// std::cout << "accelerometer: " << accelerometer.x << "\t" << accelerometer.y << "\t" << accelerometer.z << std::endl;
			std::vector<float> acc_data;
			acc_data.push_back(accelerometer.y);
			acc_data.push_back(-accelerometer.x);
			acc_data.push_back(accelerometer.z);
			imu_interpolator.addAccData(acc_data, timestamp_dbl_acc);


			// todo: check if values are there and valid!
			auto& gyroscope = imu_packet.gyroscope;

			auto timestamp_gyr = gyroscope.getTimestamp();
			auto time_since_epoch_gyr = timestamp_acc.time_since_epoch();
			auto duration_cast_gyr = std::chrono::duration_cast<std::chrono::duration<double>>(time_since_epoch_gyr);
			double timestamp_dbl_gyr = duration_cast_gyr.count();
			// std::cout << "timestamp_dbl_gyr: " << timestamp_dbl_gyr << std::endl;

			// std::cout << "gyroscope: " << gyroscope.x << "\t" << gyroscope.y << "\t" << gyroscope.z << std::endl;
			std::vector<float> gyr_data;
			gyr_data.push_back(gyroscope.y);
			gyr_data.push_back(-gyroscope.x);
			gyr_data.push_back(gyroscope.z);
			imu_interpolator.addGyrData(gyr_data, timestamp_dbl_gyr);
		} // for(auto& imu_packet : imu_data->packets) {
	} // if(size_imu_data_packets > 0) {
} // void dmvio::Luxonis::callback_imu(std::shared_ptr<dai::IMUData> imu_data) {

// ************************************************************************************
void dmvio::Luxonis::start() {
	// std::cout << "dmvio::Luxonis::start()" << std::endl;
	readCalibration();

	std::thread thread_luxonis(fnc_thread_luxonis, this);
	thread_luxonis.detach();
}

// ************************************************************************************
void dmvio::Luxonis::readCalibration() {
	if (calibrationRead)
		return;

	calibrationRead = true;
}

// ************************************************************************************
void Luxonis::setUndistorter(Undistort *undistort) {
	// std::cout << "setUndistorter" << std::endl;
	this->undistorter = undistort;
}
