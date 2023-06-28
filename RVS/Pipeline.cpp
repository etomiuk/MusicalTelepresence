/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2018, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Original authors:

Universite Libre de Bruxelles, Brussels, Belgium:
  Sarah Fachada, Sarah.Fernandes.Pinto.Fachada@ulb.ac.be
  Daniele Bonatto, Daniele.Bonatto@ulb.ac.be
  Arnaud Schenkel, arnaud.schenkel@ulb.ac.be

Koninklijke Philips N.V., Eindhoven, The Netherlands:
  Bart Kroon, bart.kroon@philips.com
  Bart Sonneveldt, bart.sonneveldt@philips.com
*/

#include "Pipeline.hpp"
#include "BlendedView.hpp"
#include "SynthesizedView.hpp"
#include "inpainting.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

///////////////
//Kinect V1

#include <opencv2/core/core.hpp>
#include <gl/glew.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <glut.h>
#include <cmath>
#include <cstdio>

#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include <opencv2\opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/video/video.hpp>



///////////////
//Lucid SDK
#include "ArenaApi.h"
#include "SaveApi.h"

//OpenCV for testing
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"

#include<iostream>
//windows The header file, must be, otherwise NuiApi.h It won't 
#include <Windows.h>
//Kinect for windows  The header file 
#include "NuiApi.h"

#include <d3d11.h>

// OpenGL Variables
const int width = 640;
const int height = 480;
// The farthest distance (mm) --> distance of the depth camera??
const int MAX_DISTANCE = 3500;
// The recent distance (mm)
const int MIN_DISTANCE = 200;
long depthToRgbMap1[width * height * 2];
long depthToRgbMap2[width * height * 2];
int inputCam;

// Kinect variables

// Depth image event handle 
HANDLE depthStream1 = NULL;
HANDLE depthStream2 = NULL;
// Color image event handle
HANDLE rgbStream1 = NULL;
HANDLE rgbStream2 = NULL;
HRESULT hr;

INuiSensor* sensor1;
INuiSensor* sensor2;
// Get the color image 1 The frame event 
HANDLE nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
// Depth image acquisition 1 The frame event 
HANDLE nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

// Get the color image 1 The frame event 
HANDLE nextColorFrameEvent2 = CreateEvent(NULL, TRUE, FALSE, NULL);
// Depth image acquisition 1 The frame event 
HANDLE nextDepthFrameEvent2 = CreateEvent(NULL, TRUE, FALSE, NULL);
///////////////////


////////////////////
// Lucid Variables
std::vector<Arena::IDevice*> RGBcams;
std::vector<Arena::IDevice*> depthCams;

Arena::IDevice* RGBcam;
Arena::IDevice* depthCam;
Arena::ISystem* pSystem; //system of cams to open when initiaizing
Arena::IImage* pImageRGB; //images pointers
Arena::IImage* pImageDepth;
GenICam::gcstring acquisitionModeInitialDepth; // to restore initial values to cameras - necessary?
GenICam::gcstring acquisitionModeInitialRGB; 



////////////////////
#if WITH_OPENGL
#include "helpersGL.hpp"
#include "RFBO.hpp"
#endif


//////////////////////////////////////////////
// Streaming
#pragma comment(lib, "Ws2_32.lib")
#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <iostream>          // For cout and cerr
#include <cstdlib>           // For atoi()
#include <map>
#include <conio.h>
#include <opencv2/videoio.hpp>

#define BUF_LEN 65540

#include "config.h"

UDPSocket socket(2000);
UDPSocket socket_pos(3000);
int jpegqual = ENCODE_QUALITY; // Compression Parameter
string sourceAddress = "132.206.162.193";
unsigned short clientPort;
cv::Mat send;
vector < uchar > encoded;
unsigned short portsend = 2001;
////////////////////////////////////////


namespace rvs
{
	Pipeline::Pipeline()
	{
#ifndef NDEBUG
		cv::setBreakOnError(true);
#endif
	}

	
	void initLucidCam() {
		// initialize devices 
		std::cout << "Initialize Lucid cameras" << std::endl;
		pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(100);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices(); //list of our devices (not pointers
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return;
		}

		/*
		//set the RGB and Depth pointers (hard-coded for now)
		GenICam::gcstring serialToFind = "224400367";
		GenICam::gcstring serialToFindDepth = "223901283";

		auto it = std::find_if(begin(deviceInfos), end(deviceInfos), [&serialToFind](Arena::DeviceInfo deviceInfo)
			{
				return deviceInfo.SerialNumber() == serialToFind;
			});

		RGBcam = pSystem->CreateDevice(*it);
		//open stream for camera
		std::cout << "Created RGB camera" << std::endl;

		auto it2 = std::find_if(begin(deviceInfos), end(deviceInfos), [&serialToFindDepth](Arena::DeviceInfo deviceInfo)
			{
				return deviceInfo.SerialNumber() == serialToFindDepth;
			});

		depthCam = pSystem->CreateDevice(*it2);
		std::cout << "Created Depth camera" << std::endl;
		*/

		//ptr array of devices
		std::vector<Arena::IDevice*> vDevices = std::vector<Arena::IDevice*>(); //make global?
		for (auto& deviceInfo : deviceInfos)
		{
			Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfo);

			//check if depth camera to assign those global pointers (will need to be modified to scale for multiple devices)
			GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
			std::string deviceModelName_tmp = deviceModelName.c_str();
			if (deviceModelName_tmp.rfind("HTP", 0) == 0) {
				depthCam = pDevice;
			}
			else {
				RGBcam = pDevice;
			}
			vDevices.push_back(pDevice);
		}

		for (auto& device : vDevices) {
			std::cout << Arena::GetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "DeviceSerialNumber").c_str() << std::endl;

			//set the node values to be able to acquire and stream images
			//make this better when scaling
			GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "DeviceModelName");
			std::string deviceModelName_tmp = deviceModelName.c_str();
			if (deviceModelName_tmp.rfind("HTP", 0) == 0) {
				acquisitionModeInitialDepth = Arena::GetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "AcquisitionMode");
			}
			else {
				acquisitionModeInitialRGB = Arena::GetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "AcquisitionMode");
			}
			

			// Set acquisition mode
			//    Set acquisition mode before starting the stream. Starting the stream
			//    requires the acquisition mode to be set beforehand. The acquisition
			//    mode controls the number of images a device acquires once the stream
			//    has been started. Setting the acquisition mode to 'Continuous' keeps
			//    the stream from stopping. This example returns the camera to its
			//    initial acquisition mode near the end of the example.
			std::cout << "Set acquisition mode to 'Continuous'\n";
			Arena::SetNodeValue<GenICam::gcstring>(
				device->GetNodeMap(),
				"AcquisitionMode",
				"Continuous");

			// Set buffer handling mode
			//    Set buffer handling mode before starting the stream. Starting the
			//    stream requires the buffer handling mode to be set beforehand. The
			//    buffer handling mode determines the order and behavior of buffers in
			//    the underlying stream engine. Setting the buffer handling mode to
			//    'NewestOnly' ensures the most recent image is delivered, even if it
			//    means skipping frames.
			std::cout << "Set buffer handling mode to 'NewestOnly'\n";
			Arena::SetNodeValue<GenICam::gcstring>(
				device->GetTLStreamNodeMap(),
				"StreamBufferHandlingMode",
				"NewestOnly");

			// Enable stream auto negotiate packet size
			//    Setting the stream packet size is done before starting the stream.
			//    Setting the stream to automatically negotiate packet size instructs
			//    the camera to receive the largest packet size that the system will
			//    allow. This generally increases frame rate and results in fewer
			//    interrupts per image, thereby reducing CPU load on the host system.
			//    Ethernet settings may also be manually changed to allow for a
			//    larger packet size.
			std::cout << "Enable stream to auto negotiate packet size\n";
			Arena::SetNodeValue<bool>(
				device->GetTLStreamNodeMap(),
				"StreamAutoNegotiatePacketSize",
				true);

			// Enable stream packet resend
			//    Enable stream packet resend before starting the stream. Images are
			//    sent from the camera to the host in packets using UDP protocol,
			//    which includes a header image number, packet number, and timestamp
			//    information. If a packet is missed while receiving an image, a
			//    packet resend is requested and this information is used to retrieve
			//    and redeliver the missing packet in the correct order.
			std::cout << "Enable stream packet resend\n";
			Arena::SetNodeValue<bool>(
				device->GetTLStreamNodeMap(),
				"StreamPacketResendEnable",
				true);

			// Start stream
			//    Start the stream before grabbing any images. Starting the stream
			//    allocates buffers, which can be passed in as an argument (default: 10),
			//    and begins filling them with data. Starting the stream blocks write
			//    access to many features such as width, height, and pixel format, as
			//    well as acquisition and buffer handling modes, among others. The stream
			//    needs to be stopped later.
			std::cout << "Start stream\n";
			device->StartStream();

			//***********************to put after we are done acquiring images
			/*
			std::cout << "Stop stream\n";
			device->StopStream();

			// return nodes to their initial values
			Arena::SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "AcquisitionMode", acquisitionModeInitial);
			*/
		}
	}

	//kinect stuff
	bool initKinect() {

		// Get a working kinect sensor
		int numSensors;

		hr = NuiCreateSensorByIndex(0, &sensor1);
		hr = NuiGetSensorCount(&numSensors);

		// Initialize sensor
		sensor1->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		hr = sensor1->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,        // Image stream flags, e.g. near mode
			2,        // Number of frames to buffer
			nextColorFrameEvent,     // Event handle
			&depthStream1);

		if (FAILED(hr))// Determine if the extraction is correct 
		{
			std::cout << "Could not open color image stream video" << std::endl;
			sensor1->NuiShutdown();
			return false;
		}

		hr = sensor1->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,      // Image stream flags, e.g. near mode
			2,      // Number of frames to buffer
			nextDepthFrameEvent,   // Event handle
			&rgbStream1);
		if (FAILED(hr))// Determine if the extraction is correct 
		{
			std::cout << "Could not open depth image stream video" << std::endl;
			sensor1->NuiShutdown();
			return false;
		}

		if (numSensors > 1) {
			// Initialize sensor
			hr = NuiCreateSensorByIndex(1, &sensor2);
			sensor2->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
			hr = sensor2->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,        // Image stream flags, e.g. near mode
				2,        // Number of frames to buffer
				nextDepthFrameEvent2,     // Event handle
				&depthStream2);

			if (FAILED(hr))// Determine if the extraction is correct
			{
				std::cout << "Could not open color image stream video in camera: 2" << std::endl;
				sensor2->NuiShutdown();
				return false;
			}

			hr = sensor2->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,      // Image stream flags, e.g. near mode
				2,      // Number of frames to buffer
				nextColorFrameEvent2,   // Event handle
				&rgbStream2);
			if (FAILED(hr))// Determine if the extraction is correct
			{
				std::cout << "Could not open depth image stream video in camera: 2" << std::endl;
				sensor2->NuiShutdown();
				return false;
			}
		}

		return sensor1;
	}


	const char* depthToStr(int depth) {
		switch (depth) {
		case CV_8U: return "unsigned char";
		case CV_8S: return "char";
		case CV_16U: return "unsigned short";
		case CV_16S: return "short";
		case CV_32S: return "int";
		case CV_32F: return "float";
		case CV_64F: return "double";
		}
		return "invalid type!";
	}

	void UDPsend(cv::Mat3f image) { //server stuff
		std::cout << "Server start" << std::endl;
		//cv::resize(image, send, cv::Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR);		
		vector < int > compression_params;
		//vector < uchar > encoded;
		//image.convertTo(send, CV_8UC1);
		cv::Mat Temp;
		image.convertTo(Temp, CV_8UC3, 255);
		cvtColor(Temp, send, cv::COLOR_BGRA2BGR);

		std::cout << image.size() << " " << image.depth() << std::endl;
		std::cout << "Image depth is : " << depthToStr(image.depth()) << endl;
		compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
		compression_params.push_back(jpegqual);

		//cv::Mat send = image.clone();

		std::cout << "Encoding" << std::endl;
		std::cout << "original image size " << send.size() << std::endl;
		if (imencode(".jpg", send, encoded, compression_params)) {
			std::cout << "Encoding success" << std::endl;
		}
		else {
			std::cout << "FAILED ENCODING" << std::endl;
		}
		cv::Mat test = cv::imdecode(encoded, cv::IMREAD_COLOR);
		//imshow("send Mat", send);
		//imshow("send", test);
		//imshow("imagetest", image);
		int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;

		int ibuf[1];
		ibuf[0] = total_pack;
		std::cout << "Server Sending to: " << sourceAddress << " Port: " << portsend << std::endl;
		socket.sendTo(ibuf, sizeof(int), sourceAddress, portsend);

		std::cout << "image Sending " << encoded.size() << " in " << total_pack << " Packets" << std::endl;
		for (int i = 0; i < total_pack; i++)
			socket.sendTo(&encoded[i * PACK_SIZE], PACK_SIZE, sourceAddress, portsend);
	}

	float* UDPrecieve_pose() { //server stuff
		//vector <unsigned char> posvalue[6*sizeof(float)];

		float headpos[6];
		unsigned char posvalue[100];
		const int numsize = sizeof(posvalue);
		socket_pos.recv(&posvalue, sizeof(posvalue));

		char decodedpos[numsize + 1];
		char* p = decodedpos;
		memcpy(decodedpos, posvalue, numsize);
		decodedpos[numsize] = '\0';        // Null-terminate the string
		//float* floatArray = reinterpret_cast<float*>(&posvalue);
		std::cout << "Head position output is: " << decodedpos << std::endl;

		size_t n = 0;

		for (int pos = 0; n < 6 && sscanf(p, "%f%n", headpos + n, &pos) == 1; p += pos)
		{
			++n;
		}
		std::cout << "Headpos Floats: ";
		for (size_t i = 0; i < n; i++)
		{
			printf("%.2f ", headpos[i]);
		}

		putchar('\n');
		return headpos;
	}

	void Pipeline::execute() //function to execute the view synthesis
	{
		auto inputFrame = 0;
		auto virtualFrame = 0;
		auto virtualView = 0;

		initLucidCam();

		
			//std::cout << "Get image from RGB" << std::endl;
			//pImageRGB = RGBcam->GetImage(2000); //2000 ms is timeout for taking image - raises error if passed timeout, we put no args and just wait forever

			//std::cout << "Get image from Depth" << std::endl;
			//pImageDepth = depthCam->GetImage(2000); //these images are not synchronized
			
			while (1) {
				auto startTimeALL = clock();
				liveView(inputFrame, virtualFrame, virtualView);
				auto executeTimeALL = double(clock() - startTimeALL) / CLOCKS_PER_SEC;
				std::cout
					<< std::endl
					<< "FULL PASS TIME: " << std::fixed << std::setprecision(3) << executeTimeALL
					<< " sec."
					<< std::endl;
			}
			//avoid memory leaks
		

			
		
		//obtain image from both cameras (not synchronized right now ummmm)
		


		//after interation of loop requeue images
		
		/* CODE TO IEW AN IMAGE USING OPENCV SOMETIMES TURNS OUT WEIRD
		std::cout << "getting nodemap" << std::endl;
		GenApi::INodeMap* pNodeMap = depthCam->GetNodeMap();
		GenICam::gcstring windowName = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceModelName");

		Arena::IImage* pImage;
		cv::Mat img;

		std::cout << "inner loop" << std::endl;
		pImage = depthCam->GetImage(2000);
		img = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC1, (void*)pImage->GetData());

		cv::imshow(windowName.c_str(), img);
		cv::waitKey(5000);
		RGBcam->RequeueBuffer(pImage);
		cv::destroyAllWindows();
		*/

		/*
		if (initKinect()) {
			while (1) {
				if ((WaitForSingleObject(nextColorFrameEvent, INFINITE) == 0) && (WaitForSingleObject(nextColorFrameEvent2, INFINITE) == 0)) { //&& (WaitForSingleObject(nextColorFrameEvent2, INFINITE) == 0)) {
					std::cout << "inner if" << std::endl;
					while (1) {
						auto startTimeALL = clock();
						liveView(inputFrame, virtualFrame, virtualView);
						auto executeTimeALL = double(clock() - startTimeALL) / CLOCKS_PER_SEC;
						std::cout
							<< std::endl
							<< "FULL PASS TIME: " << std::fixed << std::setprecision(3) << executeTimeALL
							<< " sec."
							<< std::endl;
					}
				}
				else {
					std::cout << ".";
				}
			}
		}
		else {
			std::cout << "Failed to initialize" << std::endl;
		}
		*/
	


		// clean up
		//
		std::cout << "Stop stream\n";
		RGBcam->StopStream();
		depthCam->StopStream();

		// return nodes to their initial values
		std::cout << "Reset initial values" << std::endl;
		Arena::SetNodeValue<GenICam::gcstring>(RGBcam->GetNodeMap(), "AcquisitionMode", acquisitionModeInitialRGB);
		Arena::SetNodeValue<GenICam::gcstring>(depthCam->GetNodeMap(), "AcquisitionMode", acquisitionModeInitialDepth);


		pSystem->DestroyDevice(RGBcam);
		pSystem->DestroyDevice(depthCam);
		std::cout << "Destroyed Lucid cameras" << std::endl;
		Arena::CloseSystem(pSystem);
	}

	bool Pipeline::wantColor()
	{
		return false;
	}

	bool Pipeline::wantMaskedColor()
	{
		return false;
	}

	bool Pipeline::wantMask()
	{
		return false;
	}

	bool Pipeline::wantDepth()
	{
		return false;
	}

	bool Pipeline::wantMaskedDepth()
	{
		return false;
	}

	void Pipeline::saveColor(cv::Mat3f, int, int, Parameters const&)
	{
		throw std::logic_error(std::string(__func__) + "not implemented");
	}

	void Pipeline::saveMaskedColor(cv::Mat3f, int, int, Parameters const&)
	{
		throw std::logic_error(std::string(__func__) + "not implemented");
	}

	void Pipeline::saveMask(cv::Mat1b, int, int, Parameters const&)
	{
		throw std::logic_error(std::string(__func__) + " not implemented");
	}

	void Pipeline::saveDepth(cv::Mat1f, int, int, Parameters const&)
	{
		throw std::logic_error(std::string(__func__) + " not implemented");
	}

	void Pipeline::saveMaskedDepth(cv::Mat1f, cv::Mat1b, int, int, Parameters const&)
	{
		throw std::logic_error(std::string(__func__) + " not implemented");
	}

	void Pipeline::onIntermediateSynthesisResult(int, int, int, int, SynthesizedView const&) {}

	void Pipeline::onIntermediateBlendingResult(int, int, int, int, BlendedView const&) {}

	void Pipeline::onFinalBlendingResult(int, int, int, BlendedView const&) {}

	auto getExtendedIndex(int outputFrameIndex, int numberOfInputFrames) {

		if (numberOfInputFrames <= 0) {
			throw std::runtime_error("Cannot extend frame index with zero input frames");
		}
		const auto frameGroupIndex = outputFrameIndex / numberOfInputFrames;
		const auto frameRelativeIndex = outputFrameIndex % numberOfInputFrames;
		return frameGroupIndex % 2 != 0 ? numberOfInputFrames - frameRelativeIndex - 1
			: frameRelativeIndex;

	}

	void Pipeline::computeView(int inputFrame, int virtualFrame, int virtualView) //original function
	{
		auto startTimeinit = clock();
		// Virtual view parameters for this frame and view
		auto params_virtual = getConfig().params_virtual[virtualView];
		if (!getConfig().pose_trace.empty()) {
			auto pose = getConfig().pose_trace[inputFrame];
			params_virtual.setPosition(params_virtual.getPosition() + pose.position);
			params_virtual.setRotation(pose.rotation);
			std::cout << "Pose: " << params_virtual.getPosition() << ' ' << params_virtual.getRotation() << std::endl;
		}

		// Initialize OpenGL frame buffer objects
#if WITH_OPENGL
		auto intermediateSize = cv::Size(
			int(detail::g_rescale * params_virtual.getSize().width),
			int(detail::g_rescale * params_virtual.getSize().height));
		if (g_with_opengl) {
			auto FBO = opengl::RFBO::getInstance();
			FBO->init(intermediateSize);
		}
#endif
		auto executeTimeinit = double(clock() - startTimeinit) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total initialize Time: " << std::fixed << std::setprecision(3) << executeTimeinit
			<< " sec."
			<< std::endl;
		auto startTimesetup = clock();
		// Setup a view blender
		auto blender = createBlender(virtualView);

		// Partial setup of a space transformer
		auto spaceTransformer = createSpaceTransformer(virtualView);
		spaceTransformer->set_targetPosition(&params_virtual);

		auto executeTimesetup = double(clock() - startTimesetup) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total set up  Time: " << std::fixed << std::setprecision(3) << executeTimesetup
			<< " sec."
			<< std::endl;
		auto startTimeforfront = clock();
		// For each input view
		for (auto inputView = 0u; inputView != getConfig().InputCameraNames.size(); ++inputView) {
			std::cout << getConfig().InputCameraNames[inputView] << " => " << getConfig().VirtualCameraNames[virtualView] << std::endl;
			auto const& params_real = getConfig().params_real[inputView];

			// Complete setup of space transformer
			spaceTransformer->set_inputPosition(&params_real);

			// Setup a view synthesizer
			auto synthesizer = createSynthesizer(inputView, virtualView);
			synthesizer->setSpaceTransformer(spaceTransformer.get());

			auto executeTimeforfront = double(clock() - startTimeforfront) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total for front Time: " << std::fixed << std::setprecision(3) << executeTimeforfront
				<< " sec."
				<< std::endl;
			auto startTimeload = clock();
			//posetrace longer that input view: back and forwards in the input video
			int frame_to_load = getExtendedIndex(inputFrame, getConfig().number_of_frames);
			std::cout << "loading... " << frame_to_load << std::endl;
			// Load the input image
			auto inputImage = loadInputView(frame_to_load, inputView, params_real);
			auto executeTimeload = double(clock() - startTimeload) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total Load input  Time: " << std::fixed << std::setprecision(3) << executeTimeload
				<< " sec."
				<< std::endl;
			auto startTimeinner = clock();
			// Start OpenGL instrumentation (if any)
#if WITH_OPENGL
			if (g_with_opengl) {
				opengl::rd_start_capture_frame();
			}
#endif
			auto executeTimeinner = double(clock() - startTimeinner) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total inner loop Time: " << std::fixed << std::setprecision(3) << executeTimeinner
				<< " sec."
				<< std::endl;
			auto startTimesynth = clock();
			// Synthesize view
			synthesizer->compute(*inputImage);
			onIntermediateSynthesisResult(inputFrame, inputView, virtualFrame, virtualView, *synthesizer);
			auto executeTimesynth = double(clock() - startTimesynth) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total Synthesis Time: " << std::fixed << std::setprecision(3) << executeTimesynth
				<< " sec."
				<< std::endl;
			auto startTimeblend = clock();

			// Blend with previous results
			blender->blend(*synthesizer);
			onIntermediateBlendingResult(inputFrame, inputView, virtualFrame, virtualView, *blender);
			auto executeTimeblend = double(clock() - startTimeblend) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total blending Time: " << std::fixed << std::setprecision(3) << executeTimeblend
				<< " sec."
				<< std::endl;

			// End OpenGL instrumentation (if any)
#if WITH_OPENGL
			if (g_with_opengl) {
				opengl::rd_end_capture_frame();
			}
#endif
		}
		auto startTimemaps = clock();

		onFinalBlendingResult(inputFrame, virtualFrame, virtualView, *blender);

		// Download maps from GPU
#if WITH_OPENGL
		if (g_with_opengl) {
			blender->assignFromGL2CV(intermediateSize);
		}
#endif
		auto executeTimemaps = double(clock() - startTimemaps) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total maps Time: " << std::fixed << std::setprecision(3) << executeTimemaps
			<< " sec."
			<< std::endl;
		auto startTimeinpaint = clock();

		// Perform inpainting
		cv::Mat3f color = detail::inpaint(blender->get_color(), blender->get_inpaint_mask(), true);
		auto executeTimeinpaint = double(clock() - startTimeinpaint) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total inpainting Time: " << std::fixed << std::setprecision(3) << executeTimeinpaint
			<< " sec."
			<< std::endl;
		auto startTimedown = clock();

		// Downscale (when g_Precision != 1)
		resize(color, color, params_virtual.getSize());
		auto executeTimedown = double(clock() - startTimedown) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total downscale Time: " << std::fixed << std::setprecision(3) << executeTimedown
			<< " sec."
			<< std::endl;
		auto startTimesave = clock();

		// Write regular output (activated by OutputFiles)
		if (wantColor()) {
			saveColor(color, virtualFrame, virtualView, params_virtual);
		}

		// Compute mask (activated by OutputMasks or MaskedOutputFiles or MaskedDepthOutputFiles)
		cv::Mat1b mask;
		if (wantMask() || wantMaskedColor() || wantMaskedDepth()) {
			mask = blender->get_validity_mask(getConfig().validity_threshold);
			resize(mask, mask, params_virtual.getSize(), cv::INTER_NEAREST);
		}

		// Write mask (activated by OutputMasks)
		if (wantMask()) {
			saveMask(mask, virtualFrame, virtualView, params_virtual);
		}

		// Write masked output (activated by MaskedOutputFiles)
		if (wantMaskedColor()) {
			color.setTo(cv::Vec3f::all(0.5f), mask);
			saveMaskedColor(color, virtualFrame, virtualView, params_virtual);
		}

		// Write depth maps (activated by DepthOutputFiles)
		if (wantDepth()) {
			auto depth = blender->get_depth();
			resize(depth, depth, params_virtual.getSize());
			saveDepth(depth, virtualFrame, virtualView, params_virtual);
		}

		// Write masked depth maps (activated by MaskedDepthOutputFiles)
		if (wantMaskedDepth()) {
			auto depth = blender->get_depth();
			resize(depth, depth, params_virtual.getSize());
			saveMaskedDepth(depth, mask, virtualFrame, virtualView, params_virtual);
		}

#if WITH_OPENGL
		if (g_with_opengl) {
			auto FBO = opengl::RFBO::getInstance();
			FBO->free();
		}
#endif
		clock_t last_cycle = clock();

		auto executeTimesave = double(clock() - startTimesave) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total Save and end Time: " << std::fixed << std::setprecision(3) << executeTimesave
			<< " sec."
			<< std::endl;
	}

	std::unique_ptr<BlendedView> Pipeline::createBlender(int)
	{
		if (getConfig().blending_method == BlendingMethod::simple) {
			return std::unique_ptr<BlendedView>(new BlendedViewSimple(getConfig().blending_factor));
		}

		if (getConfig().blending_method == BlendingMethod::multispectral) {
			return std::unique_ptr<BlendedView>(new BlendedViewMultiSpec(getConfig().blending_low_freq_factor, getConfig().blending_high_freq_factor));
		}

		std::ostringstream what;
		what << "Unknown view blending method \"" << getConfig().blending_method << "\"";
		throw std::runtime_error(what.str());
	}

	std::unique_ptr<SynthesizedView> Pipeline::createSynthesizer(int, int)
	{
		if (getConfig().vs_method == ViewSynthesisMethod::triangles) {
			return std::unique_ptr<SynthesizedView>(new SynthetisedViewTriangle);
		}

		std::ostringstream what;
		what << "Unknown view synthesis method \"" << getConfig().vs_method << "\"";
		throw std::runtime_error(what.str());
	}

	std::unique_ptr<SpaceTransformer> Pipeline::createSpaceTransformer(int)
	{
#if WITH_OPENGL
		if (g_with_opengl) {
			return std::unique_ptr<SpaceTransformer>(new OpenGLTransformer);
		}
#endif
		return std::unique_ptr<SpaceTransformer>(new GenericTransformer);
	}

	//////////////////////////////////////////////////////////////////////
	//Kinect mod
	//////////////////////////////////////////////////////////////////////

	void Pipeline::liveView(int inputFrame, int virtualFrame, int virtualView) //(int inputFrame, int virtualFrame, int virtualView)
	{
		auto startTimeinit = clock();
		// MODIFIED FOR TESTING ***************************************************************
		//float* headposition = UDPrecieve_pose(); //idk what to do with this
		//cv::Vec3f pos = { headposition[0], headposition[1] , headposition[2] };
		//cv::Vec3f rot = { headposition[3], headposition[4] , headposition[5] };

		cv::Vec3f pos = { 0, 0 , 0 };
		cv::Vec3f rot = { 0, -90, 0 };

		// Virtual view parameters for this frame and view
		auto params_virtual = getConfig().params_virtual[virtualView];
		//if (!getConfig().pose_trace.empty()) {
		auto pose = getConfig().pose_trace[inputFrame];
		//if (pos[2] > 6) {
		//	pos[2] = 6;
		//}
		pose.position = pos;
		pose.rotation = rot;
		params_virtual.setPosition(params_virtual.getPosition() + pose.position);
		params_virtual.setRotation(pose.rotation);
		std::cout << "Pose: " << params_virtual.getPosition() << ' ' << params_virtual.getRotation() << std::endl;
		//}

		// Initialize OpenGL frame buffer objects
#if WITH_OPENGL
		auto intermediateSize = cv::Size(
			int(detail::g_rescale * params_virtual.getSize().width),
			int(detail::g_rescale * params_virtual.getSize().height));
		if (g_with_opengl) {
			auto FBO = opengl::RFBO::getInstance();
			FBO->init(intermediateSize);
		}
#endif		
		auto executeTimeinit = double(clock() - startTimeinit) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total initialize Time: " << std::fixed << std::setprecision(3) << executeTimeinit
			<< " sec."
			<< std::endl;
		auto startTimesetup = clock();

		// Setup a view blender
		auto blender = createBlender(virtualView);

		// Partial setup of a space transformer
		auto spaceTransformer = createSpaceTransformer(virtualView);

		spaceTransformer->set_targetPosition(&params_virtual);

		auto executeTimesetup = double(clock() - startTimesetup) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total set up  Time: " << std::fixed << std::setprecision(3) << executeTimesetup
			<< " sec."
			<< std::endl;
		auto startTimeforfront = clock();
		// For each input view
		std::cout << getConfig().InputCameraNames.size() << std::endl;
		for (auto inputView = 0u; inputView != getConfig().InputCameraNames.size(); ++inputView) {
			inputCam = inputView;
			std::cout << getConfig().InputCameraNames[inputView] << " => " << getConfig().VirtualCameraNames[virtualView] << std::endl;
			auto const& params_real = getConfig().params_real[inputView];

			// Complete setup of space transformer
			spaceTransformer->set_inputPosition(&params_real);

			// Setup a view synthesizer
			auto synthesizer = createSynthesizer(inputView, virtualView);
			synthesizer->setSpaceTransformer(spaceTransformer.get());

			auto executeTimeforfront = double(clock() - startTimeforfront) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total for front Time: " << std::fixed << std::setprecision(3) << executeTimeforfront
				<< " sec."
				<< std::endl;
			auto startTimeload = clock();
			//posetrace longer that input view: back and forwards in the input video
			int frame_to_load = getExtendedIndex(inputFrame, getConfig().number_of_frames);
			std::cout << "loading... " << frame_to_load << std::endl;
			// Load the input image
			auto inputImage = loadInputView(frame_to_load, inputView, params_real);
			// Grab frames from kinect
			//auto inputImage = getKinectData(inputView, params_real);
			auto executeTimeload = double(clock() - startTimeload) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total Load input  Time: " << std::fixed << std::setprecision(3) << executeTimeload
				<< " sec."
				<< std::endl;
			auto startTimeinner = clock();
			// Start OpenGL instrumentation (if any)
#if WITH_OPENGL
			if (g_with_opengl) {
				opengl::rd_start_capture_frame();
			}
#endif
			auto executeTimeinner = double(clock() - startTimeinner) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total inner loop Time: " << std::fixed << std::setprecision(3) << executeTimeinner
				<< " sec."
				<< std::endl;
			auto startTimesynth = clock();
			// Synthesize view
			synthesizer->compute(*inputImage);
			onIntermediateSynthesisResult(inputFrame, inputView, virtualFrame, virtualView, *synthesizer);
			auto executeTimesynth = double(clock() - startTimesynth) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total Synthesis Time: " << std::fixed << std::setprecision(3) << executeTimesynth
				<< " sec."
				<< std::endl;
			auto startTimeblend = clock();

			// Blend with previous results
			blender->blend(*synthesizer);
			onIntermediateBlendingResult(inputFrame, inputView, virtualFrame, virtualView, *blender);
			auto executeTimeblend = double(clock() - startTimeblend) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total blending Time: " << std::fixed << std::setprecision(3) << executeTimeblend
				<< " sec."
				<< std::endl;

			// End OpenGL instrumentation (if any)
#if WITH_OPENGL
			if (g_with_opengl) {
				opengl::rd_end_capture_frame();
			}
#endif
		}
		auto startTimemaps = clock();

		onFinalBlendingResult(inputFrame, virtualFrame, virtualView, *blender);

		// Download maps from GPU
#if WITH_OPENGL
		if (g_with_opengl) {
			blender->assignFromGL2CV(intermediateSize);
		}
#endif
		auto executeTimemaps = double(clock() - startTimemaps) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total maps Time: " << std::fixed << std::setprecision(3) << executeTimemaps
			<< " sec."
			<< std::endl;
		auto startTimeinpaint = clock();

		// Perform inpainting
		cv::Mat3f color = detail::inpaint(blender->get_color(), blender->get_inpaint_mask(), true);
		auto executeTimeinpaint = double(clock() - startTimeinpaint) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total inpainting Time: " << std::fixed << std::setprecision(3) << executeTimeinpaint
			<< " sec."
			<< std::endl;
		auto startTimedown = clock();

		// Downscale (when g_Precision != 1)
		resize(color, color, params_virtual.getSize());
		auto executeTimedown = double(clock() - startTimedown) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Total downscale Time: " << std::fixed << std::setprecision(3) << executeTimedown
			<< " sec."
			<< std::endl;
		auto startTimesave = clock();

		if (color.data == NULL) {
			std::cout << "Output NULL" << std::endl;
		}
		else {
			cv::imshow("Lucid RVS Output Test", color);
			cv::waitKey(1);
			/*
			//Send image to server
			UDPsend(color);

			auto executeTimesave = double(clock() - startTimesave) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Total Send and end Time: " << std::fixed << std::setprecision(3) << executeTimesave
				<< " sec."
				<< std::endl;
			//cv::waitKey(5);
			*/
		}

		/*
		std::cout << "Server start" << std::endl;
		cv::resize(color, send, cv::Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR);
		vector < uchar > encoded;
		vector < int > compression_params;
		compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
		compression_params.push_back(jpegqual);

		std::cout << "Encoding" << std::endl;
		imencode(".jpg", send, encoded, compression_params);
		//imshow("send", send);
		int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;

		int ibuf[1];
		ibuf[0] = total_pack;
		std::cout << "Server Sending to: " << sourceAddress << " Port: " << portsend << std::endl;
		socket.sendTo(ibuf, sizeof(int), sourceAddress, portsend);

		std::cout << "image Sending in : " << total_pack << " Packets" << std::endl;
		for (int i = 0; i < total_pack; i++)
			socket.sendTo(&encoded[i * PACK_SIZE], PACK_SIZE, sourceAddress, portsend);
		*/

		//cv::waitKey(FRAME_INTERVAL);

		//clock_t next_cycle = clock();
		//double duration = (next_cycle - last_cycle) / (double)CLOCKS_PER_SEC;
		//cout << "\teffective FPS:" << (1 / duration) << " \tkbps:" << (PACK_SIZE * total_pack / duration / 1024 * 8) << endl;

		//cout << next_cycle - last_cycle;
		//last_cycle = next_cycle;

		/*
		// Write regular output (activated by OutputFiles)
		if (wantColor()) {
			saveColor(color, virtualFrame, virtualView, params_virtual);
		}

		// Compute mask (activated by OutputMasks or MaskedOutputFiles or MaskedDepthOutputFiles)
		cv::Mat1b mask;
		if (wantMask() || wantMaskedColor() || wantMaskedDepth()) {
			mask = blender->get_validity_mask(getConfig().validity_threshold);
			resize(mask, mask, params_virtual.getSize(), cv::INTER_NEAREST);
		}

		// Write mask (activated by OutputMasks)
		if (wantMask()) {
			saveMask(mask, virtualFrame, virtualView, params_virtual);
		}

		// Write masked output (activated by MaskedOutputFiles)
		if (wantMaskedColor()) {
			color.setTo(cv::Vec3f::all(0.5f), mask);
			saveMaskedColor(color, virtualFrame, virtualView, params_virtual);
		}

		// Write depth maps (activated by DepthOutputFiles)
		if (wantDepth()) {
			auto depth = blender->get_depth();
			resize(depth, depth, params_virtual.getSize());
			saveDepth(depth, virtualFrame, virtualView, params_virtual);
		}

		// Write masked depth maps (activated by MaskedDepthOutputFiles)
		if (wantMaskedDepth()) {
			auto depth = blender->get_depth();
			resize(depth, depth, params_virtual.getSize());
			saveMaskedDepth(depth, mask, virtualFrame, virtualView, params_virtual);
		}
		*/
#if WITH_OPENGL
		if (g_with_opengl) {
			auto FBO = opengl::RFBO::getInstance();
			FBO->free();
		}
#endif
		//auto executeTimesave = double(clock() - startTimesave) / CLOCKS_PER_SEC;
		//std::cout
		//	<< std::endl
		//	<< "Total Save and end Time: " << std::fixed << std::setprecision(3) << executeTimesave
		//	<< " sec."
		//	<< std::endl;
	}
}