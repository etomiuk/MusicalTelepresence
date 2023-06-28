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

#include<iostream>
//windows The header file, must be, otherwise NuiApi.h It won't 
#include <Windows.h>
//Kinect for windows  The header file 
#include "NuiApi.h"

#include <d3d11.h>

// OpenGL Variables
const int width = 640;
const int height = 480;
// The farthest distance (mm)
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
//Kinect v2
/*
#include <Kinect.h>

//const int rgb_width = 1920;
//const int rgb_height = 1080;

//const int depth_width = 1920;
//const int depth_height = 1080;

// Kinect variables
IKinectSensor* sensor;         // Kinect sensor
IMultiSourceFrameReader* reader;     // Kinect color data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates
*/

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

		/*
		HRESULT hr;
		// Get a working kinect sensor
		int numSensors;
		//if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
		//NuiGetSensorCount(&numSensors);				

		hr = NuiGetSensorCount(&numSensors);

		// Initialize sensor
		hr = NuiCreateSensorByIndex(0, &sensor1);
		std::cout << "create sensor " << hr << std::endl;

		sensor1->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		
		hr = sensor1->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,        // Image stream flags, e.g. near mode
			2,        // Number of frames to buffer
			nextDepthFrameEvent,     // Event handle
			&depthStream1);

		if (FAILED(hr))// Determine if the extraction is correct 
		{
			std::cout << "Could not open color image stream video in camera: 1" << std::endl;
			sensor1->NuiShutdown();
		    return hr;
		}

		hr = sensor1->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,      // Image stream flags, e.g. near mode
			2,      // Number of frames to buffer
			nextColorFrameEvent,   // Event handle
			&rgbStream1);
		if (FAILED(hr))// Determine if the extraction is correct 
		{
			std::cout << "Could not open depth image stream video in camera: 1" << std::endl;
			sensor1->NuiShutdown();
			return hr;
		}

		std::cout << "number of sensors: " << numSensors << std::endl;
		if (numSensors > 1) {/*
			// Initialize sensor
			hr = NuiCreateSensorByIndex(0, &sensor2);
			sensor2->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
			hr = sensor2->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,        // Image stream flags, e.g. near mode
				2,        // Number of frames to buffer
				NULL,     // Event handle
				&depthStream2);

			if (FAILED(hr))// Determine if the extraction is correct 
			{
				std::cout << "Could not open color image stream video in camera: 2" << std::endl;
				sensor2->NuiShutdown();
				return hr;
			}

			hr = sensor2->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,      // Image stream flags, e.g. near mode
				2,      // Number of frames to buffer
				NULL,   // Event handle
				&rgbStream2);
			if (FAILED(hr))// Determine if the extraction is correct 
			{
				std::cout << "Could not open depth image stream video in camera: 2" << std::endl;
				sensor2->NuiShutdown();
				return hr;
			}
			
		}
	
	
		return sensor1;
		*/

		// Kinect V2
		/*
		if (FAILED(GetDefaultKinectSensor(&sensor))) {
			return false;
		}
		if (sensor) {
			sensor->get_CoordinateMapper(&mapper);

			sensor->Open();
			sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,&reader);
			cv::waitKey(10);
			return reader;
		}
		else {
			return false;
		}
		*/
	}
	/*
	void initServer(unsigned short sourcePort, string sourceAddress) {
		
		
		int jpegqual = ENCODE_QUALITY;
		cv::Mat frame, send;
		
		sourceAddress = "10.122.188.8";
		char connectionType, mode;
		char buffer[BUF_LEN];
		int recvMsgSize;
		std::size_t received;
		
		std::string text = "Connected to: ";
		clock_t last_cycle = clock();
		vector < uchar > encoded;

		unsigned short port;				
		port = 2000;
		
		UDPSocket socket(port);		
	}*/
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

	void UDPsend(cv::Mat3f image) {	
		std::cout << "Server start" << std::endl;
		//cv::resize(image, send, cv::Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR);		
		vector < int > compression_params;
		//vector < uchar > encoded;
		//image.convertTo(send, CV_8UC1);
		cv::Mat Temp;
		image.convertTo(Temp, CV_8UC3,255);		
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
	
	float *UDPrecieve_pose() {
		//vector <unsigned char> posvalue[6*sizeof(float)];

		float headpos[6];
		unsigned char posvalue[100];
		const int numsize = sizeof(posvalue);
		socket_pos.recv(&posvalue, sizeof(posvalue));

		char decodedpos[numsize + 1];
		char *p = decodedpos;
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

	void Pipeline::execute()
	{
		auto inputFrame = 0;
		auto virtualFrame = 0;
		auto virtualView = 0;		
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

		//Original RVS
		/*
		for (auto virtualFrame = 0; virtualFrame < getConfig().number_of_output_frames; ++virtualFrame) {
			auto inputFrame = getConfig().start_frame + virtualFrame;
			if (getConfig().number_of_output_frames > 1) {
				std::cout << std::string(5, '=') << " FRAME " << inputFrame << ' ' << std::string(80, '=') << std::endl;
			}
			for (auto virtualView = 0u; virtualView != getConfig().VirtualCameraNames.size(); ++virtualView) {
				//computeView(inputFrame, virtualFrame, virtualView);
				liveView(inputFrame, virtualFrame, virtualView);
			}
		}*/
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

	void Pipeline::computeView(int inputFrame, int virtualFrame, int virtualView)
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
			int(detail::g_rescale*params_virtual.getSize().width),
			int(detail::g_rescale*params_virtual.getSize().height));
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
		//float *headposition = UDPrecieve_pose();
		//v::Vec3f pos = { headposition[0], headposition[1] , headposition[2] };
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
			cv::imshow("Kinect RVS Output Test", color);

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
