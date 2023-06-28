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

#include "image_loading.hpp"
#include "Config.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <limits>

#include <iomanip>

///////////////
//Kinect
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

//Lucid SDK
#include "ArenaApi.h"
#include "SaveApi.h"


// OpenGL Variables
const int width = 640;
const int height = 480;
// The farthest distance (mm)
const int MAX_DISTANCE = 3500;
// The recent distance (mm)
const int MIN_DISTANCE = 200;
extern int inputCam;
//long depthToRgbMap[width * height * 2];
// We'll be using buffer objects to store the kinect point cloud

// Kinect variables
// Depth image event handle 
extern HANDLE depthStream1;
extern HANDLE depthStream2;
//HANDLE depthStream = NULL;
// Color image event handle
extern HANDLE rgbStream1;
extern HANDLE rgbStream2;
//HANDLE rgbStream = NULL;
extern INuiSensor* sensor1;
extern INuiSensor* sensor2;
//INuiSensor* sensor;
extern HRESULT hr;
// Get the color image 1 The frame event 
extern HANDLE nextColorFrameEvent; 
// Depth image acquisition 1 The frame event 
extern HANDLE nextDepthFrameEvent;

//Lucid variables (extern because declared in Pipeline)
extern Arena::IDevice* RGBcam;
extern Arena::IDevice* depthCam;
extern Arena::ISystem* pSystem; //system of cams to open when initiaizing
extern Arena::IImage* pImageRGB; //images pointers
extern Arena::IImage* pImageDepth;
///////////////////
// Kinect V2
/*
#include <Kinect.h>

int rgb_width = 1920;
int rgb_height = 1080;

int depth_width = 0;
int depth_height = 0;

// Kinect variables
extern IKinectSensor* sensor;         // Kinect sensor
extern IMultiSourceFrameReader* reader;     // Kinect color data source
extern ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates
*/
/////////////////////////


namespace rvs
{
	namespace
	{
		using detail::ColorSpace;
		using detail::g_color_space;

		void read_raw(std::ifstream& stream, cv::Mat image)
		{
			CV_Assert(stream.good() && !image.empty() && image.isContinuous());
			stream.read(reinterpret_cast<char*>(image.data), image.size().area() * image.elemSize());
		}

	//////////////////////////////////////////////////////////////////////
	//Kinect mod
	//////////////////////////////////////////////////////////////////////

	//initialize the kinect camera
		/*
		bool initKinect() {
			// Get a working kinect sensor
			int numSensors;
			if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
			if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

			hr = NuiGetSensorCount(&numSensors);

			// Initialize sensor
			sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
			hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,        // Image stream flags, e.g. near mode
				2,        // Number of frames to buffer
				nextColorFrameEvent,     // Event handle
				&depthStream);

			if (FAILED(hr))// Determine if the extraction is correct 
			{
				std::cout << "Could not open color image stream video" << std::endl;
				sensor->NuiShutdown();
				return hr;
			}

			hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
				NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
				0,      // Image stream flags, e.g. near mode
				2,      // Number of frames to buffer
				nextDepthFrameEvent,   // Event handle
				&rgbStream);
			if (FAILED(hr))// Determine if the extraction is correct 
			{
				std::cout << "Could not open depth image stream video" << std::endl;
				sensor->NuiShutdown();
				return hr;
			}
			return sensor;
		}
		*/
		//Get Detph image 
		cv::Mat getDepthDataLucid() {
			cv::Mat depth;

			//what to do with input cams numbers??
			//std::cout << "Getting image from depth ..." << std::endl;
			pImageDepth = depthCam->GetImage(2000);
			//std::cout << "Convert image to " << GetPixelFormatName(BGR8) << "\n";
			//auto pConverted = Arena::ImageFactory::Convert(pImageRGB, BGR8);

			//stored image as matrix
			depth = cv::Mat((int)pImageDepth->GetHeight(), (int)pImageDepth->GetWidth(), CV_8UC1, (void*)pImageDepth->GetData()); //might need to change dimensions
			//std::cout << "Converted depth image to cv::Mat" << std::endl;

			if (depth.data == NULL) {
				std::cout << "depth is null" << std::endl;
			}

			cv::resize(depth, depth, cv::Size(640, 480), cv::INTER_LINEAR);
			//std::cout << "resized image" << std::endl;
			//cv::imshow("RGB image", color);
			//cv::waitKey(0);
			//cv::destroyAllWindows(); 

			depthCam->RequeueBuffer(pImageDepth);
			//requeue image to avoid weird things

			return depth;
		}

		cv::Mat getDepthData() {
			// Kinect V2
			/*
			cv::Mat depth;
			
			HRESULT hr;

			IDepthFrameReader* m_pFrameReader = NULL;
			IDepthFrameSource* pFrameSource = NULL;

			hr = sensor->get_DepthFrameSource(&pFrameSource);

			if (FAILED(hr))
			{
				std::cout << "get_DepthFrameSource error" << std::endl;				
			}

			hr = pFrameSource->OpenReader(&m_pFrameReader);

			if (FAILED(hr))
			{
				std::cout << "OpenReader depth error" << std::endl;				
			}

			if (pFrameSource != NULL)
			{
				pFrameSource->Release();
				pFrameSource = NULL;
			}

			IDepthFrame* pFrame = NULL;
			hr = m_pFrameReader->AcquireLatestFrame(&pFrame);

			if (FAILED(hr))
			{
				std::cout << "AcquireLatestFrame depth error" << std::endl;				
			}

			IFrameDescription* pDescription = NULL;
			hr = pFrame->get_FrameDescription(&pDescription);

			if (FAILED(hr))
			{
				std::cout << "get_FrameDescription depth error" << std::endl;				
			}

			hr = pDescription->get_Width(&depth_width);

			if (FAILED(hr))
			{
				std::cout << "get_Width depth error" << std::endl;				
			}

			hr = pDescription->get_Height(&depth_height);

			if (FAILED(hr))
			{
				std::cout << "get_Height depth error" << std::endl;				
			}

			if (pDescription != NULL)
			{
				pDescription->Release();
				pDescription = NULL;
			}			

			UINT nBufferSize = 0; // Number of bytes in pBuffer
			UINT16* pBuffer = NULL;
			hr = pFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

			if (FAILED(hr))
			{
				std::cout << "AccessUnderlyingBuffer  depth error" << std::endl;				
			}

			depth= cv::Mat(depth_height, depth_width, CV_16UC1, pBuffer).clone();

			cv::Mat float_frame_content(depth_height, depth_width, CV_32FC1);

			for (int j = 0; j < depth.rows; j++)
				for (int i = 0; i < depth.cols; i++)
					float_frame_content.at<float>(j, i) = depth.at<UINT16>(j, i) / 65535.0f;

			depth = float_frame_content.clone();

			if (pFrame != NULL)
			{
				pFrame->Release();
				pFrame = NULL;
			}
			if (m_pFrameReader != NULL)
			{
				m_pFrameReader->Release();
				m_pFrameReader = NULL;
			}


			return depth;
			*/
			
			BOOL nearMode;
			cv::Mat depth;
			depth.create(480, 640, CV_8UC1);
			INuiFrameTexture* pColorToDepthTexture;
			NUI_IMAGE_FRAME pImageFrame_depth;
			if (inputCam == 0) {
				hr = sensor1->NuiImageStreamGetNextFrame(depthStream1, 0, &pImageFrame_depth);
				if (FAILED(hr))
				{
					std::cout << "hr: " << hr << std::endl;
					std::cout << "Could not get depth image" << std::endl;
					NuiShutdown();
				}
				hr = sensor1->NuiImageFrameGetDepthImagePixelFrameTexture(
					depthStream1, &pImageFrame_depth, &nearMode, &pColorToDepthTexture);
			}
			else if (inputCam == 1) {
				hr = sensor2->NuiImageStreamGetNextFrame(depthStream2, 0, &pImageFrame_depth);
				if (FAILED(hr))
				{
					std::cout << "hr: " << hr << std::endl;
					std::cout << "Could not get depth image" << std::endl;
					NuiShutdown();
				}
				hr = sensor2->NuiImageFrameGetDepthImagePixelFrameTexture(
					depthStream2, &pImageFrame_depth, &nearMode, &pColorToDepthTexture);
			}									

			INuiFrameTexture* pTexture = pImageFrame_depth.pFrameTexture;
			NUI_LOCKED_RECT lockedRect;
			NUI_LOCKED_RECT ColorToDepthLockRect;

			pTexture->LockRect(0, &lockedRect, NULL, 0);
			pColorToDepthTexture->LockRect(0, &ColorToDepthLockRect, NULL, 0);

			if (lockedRect.Pitch != 0){
			// Belong to the 1 the 
				for (int i = 0; i < depth.rows; i++)
				{
					uchar* prt = depth.ptr<uchar>(i);
					uchar* pBuffer = (uchar*)(lockedRect.pBits) + i * lockedRect.Pitch;
					// You need to convert here, because each depth data is 2 Bytes, should be BYTE into USHORT
					USHORT* pBufferRun = (USHORT*)pBuffer;

					for (int j = 0; j < depth.cols; j++)
					{
						// Let's go first. Let's go back 1 To the depth of the distance in 300mm-3500mm Range of pixels, mapped to [ 0 - 255 】, 
						// Out of range, all to do are edge pixels 
						if (pBufferRun[j] << 3 > MAX_DISTANCE) prt[j] = 255;
						else if (pBufferRun[j] << 3 < MIN_DISTANCE) prt[j] = 0;
						else prt[j] = (BYTE)(256 * (pBufferRun[j] << 3) / MAX_DISTANCE);
					}
				}
				if (depth.data == NULL) {
						std::cout << "color is null" << std::endl;
				}
				else {
					// unlocked 
					pTexture->UnlockRect(0);
					// Release the frame 
					if (inputCam == 0) {
						sensor1->NuiImageStreamReleaseFrame(depthStream1, &pImageFrame_depth);
					}
					else if (inputCam == 1) {
						sensor2->NuiImageStreamReleaseFrame(depthStream2, &pImageFrame_depth);
					}					
					return depth;
				}
			}
			else
			{
				std::cout << "Buffer length of received texture is bogus\r\n" << std::endl;
			}
			
		}

		cv::Mat getRgbDataLucid() {

			cv::Mat color;

			//what to do with input cams numbers??
			//std::cout << "Getting image from RGB ..." << std::endl;
			pImageRGB = RGBcam->GetImage(2000);
			//std::cout << "Convert image to " << GetPixelFormatName(BGR8) << "\n";
			auto pConverted = Arena::ImageFactory::Convert(pImageRGB, BGR8);

			//stored image as matrix
			color = cv::Mat((int)pConverted->GetHeight(), (int)pConverted->GetWidth(), CV_8UC3, (void*)pConverted->GetData()); //might need to change dimensions
			//std::cout << "Converted RGB image to cv::Mat" << std::endl;

			if (color.data == NULL) {
				std::cout << "color is null" << std::endl;
			}

			cv::resize(color, color, cv::Size(640, 480), cv::INTER_LINEAR);
			//std::cout << "resized image" << std::endl;
			//cv::imshow("RGB image", color);
			//cv::waitKey(0);
			//cv::destroyAllWindows(); 

			RGBcam->RequeueBuffer(pImageRGB);
			//requeue image to avoid weird things

			return color;
		}

		// Get RGB Image 
		cv::Mat getRgbData() {
			// Kinect V2
			/*
			cv::Mat color;
			//color.create(480, 640, CV_8UC3);
			IColorFrameReader* m_pFrameReader = NULL;
			IColorFrameSource* pFrameSource = NULL;

			hr = sensor->get_ColorFrameSource(&pFrameSource);

			if (FAILED(hr))
			{
				std::cout << "get_ColorFrameSource error" << std::endl;				
			}

			hr = pFrameSource->OpenReader(&m_pFrameReader);

			if (FAILED(hr))
			{
				std::cout << "OpenReader rgb error" << std::endl;				
			}

			if (pFrameSource != NULL)
			{
				pFrameSource->Release();
				pFrameSource = NULL;
			}			

			IColorFrame* pFrame = NULL;
			hr = m_pFrameReader->AcquireLatestFrame(&pFrame);

			if (FAILED(hr))
			{
				std::cout << "AcquireLatestFrame rgb error" << std::endl;				
			}

			IFrameDescription* pDescription = NULL;
			hr = pFrame->get_FrameDescription(&pDescription);

			if (FAILED(hr))
			{
				std::cout << "get_FrameDescription rgb error" << std::endl;				
			}

			hr = pDescription->get_Width(&rgb_width);

			if (FAILED(hr))
			{
				std::cout << "get_Width rgb error" << std::endl;				
			}

			hr = pDescription->get_Height(&rgb_height);

			if (FAILED(hr))
			{
				std::cout << "get_Height rgb error" << std::endl;				
			}

			if (pDescription != NULL)
			{
				pDescription->Release();
				pDescription = NULL;
			}

			UINT nBufferSize = 0; // Number of bytes in pBuffer
			BYTE* pBuffer = NULL;
			hr = pFrame->AccessRawUnderlyingBuffer(&nBufferSize, &pBuffer);

			if (FAILED(hr))
			{
				std::cout << "AccessRawUnderlyingBuffer rgb error" << std::endl;
				
			}

			// check out convertTo member function
			//cvtColor(in, out, CV_BGRA2BGR);
			color = cv::Mat(rgb_height, rgb_width, CV_8UC4, pBuffer).clone();

			cv::cvtColor(color, color, cv::COLOR_RGBA2BGR);

			if (pFrame != NULL)
			{
				pFrame->Release();
				pFrame = NULL;
			}
			if (m_pFrameReader != NULL)
			{
				m_pFrameReader->Release();
				m_pFrameReader = NULL;
			}

			return color;
			*/
			
			cv::Mat color;
			color.create(480, 640, CV_8UC3);
				
			NUI_IMAGE_FRAME pImageFrame_rgb;
			if (inputCam == 0) {
				// Gets the frame data from the stream handle that just opened the data stream, and the read data address is stored at pImageFrame
				hr = sensor1->NuiImageStreamGetNextFrame(rgbStream1, 0, &pImageFrame_rgb);
				if (FAILED(hr))
				{
					std::cout << "Could not get color image" << std::endl;
					sensor1->NuiShutdown();
				}
			}
			else if (inputCam == 1) {
				// Gets the frame data from the stream handle that just opened the data stream, and the read data address is stored at pImageFrame
				hr = sensor2->NuiImageStreamGetNextFrame(rgbStream2, 0, &pImageFrame_rgb);
				if (FAILED(hr))
				{
					std::cout << "Could not get color image" << std::endl;
					sensor2->NuiShutdown();
				}
			}

			INuiFrameTexture* pTexture = pImageFrame_rgb.pFrameTexture;
			NUI_LOCKED_RECT lockedRect;

			pTexture->LockRect(0, &lockedRect, NULL, 0);
			// Verify that the data obtained is valid 
			if (lockedRect.Pitch != 0)
			{
				// Convert the data to OpenCV the Mat format 
				for (int i = 0; i < color.rows; i++)
				{
					// The first i The pointer 
					uchar* prt = color.ptr(i);

					// Each byte represents 1 Color information, direct use uchar
					uchar* pBuffer = (uchar*)(lockedRect.pBits) + i * lockedRect.Pitch;

					for (int j = 0; j < color.cols; j++)
					{
						prt[3 * j] = pBuffer[4 * j];// The internal data is 4 Bytes, 0-1-2 is BGR In the first 4 None is currently in use 
						prt[3 * j + 1] = pBuffer[4 * j + 1];
						prt[3 * j + 2] = pBuffer[4 * j + 2];
					}
				}

				if (color.data == NULL) {
					std::cout << "color is null" << std::endl;
				}
				else {							
					// unlocked 
					pTexture->UnlockRect(0);
					// Release the frame 
					if (inputCam == 0) {
						sensor1->NuiImageStreamReleaseFrame(rgbStream1, &pImageFrame_rgb);
					}
					else {
						sensor2->NuiImageStreamReleaseFrame(rgbStream2, &pImageFrame_rgb);
					}					
					return color;
				}
			}
			else
			{
				std::cout << "Buffer length of received texture is bogus\r\n" << std::endl;
			}
			if (color.data == NULL) {
				std::cout << "color is null" << std::endl;
			}
			else {						
				return color;
			}			
			
		}
		/// /////////////////////////////////////////////////////////////////////////////////////////////

		cv::Mat read_color_YUV(std::string filepath, int frame, Parameters const& parameters) {

			auto startTimeinit = clock();

			auto size = parameters.getPaddedSize();
			auto bit_depth = parameters.getColorBitDepth();
			auto type = CV_MAKETYPE(cvdepth_from_bit_depth(bit_depth), 1);
			cv::Mat y_channel(size, type);
			cv::Mat u_channel(size / 2, type);
			cv::Mat v_channel(size / 2, type);

			std::ifstream stream(filepath, std::ios::binary);
			if (!stream.good()) {
				std::ostringstream what;
				what << "Failed to read raw YUV color file \"" << filepath << "\"";
				throw std::runtime_error(what.str());
			}

			auto executeTimeinit = double(clock() - startTimeinit) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Read Color Total Time initialize: " << std::fixed << std::setprecision(3) << executeTimeinit
				<< " sec."
				<< std::endl;

			auto startTimeread = clock();

			stream.seekg(size.area() * y_channel.elemSize() * 3 / 2 * frame);
			read_raw(stream, y_channel);
			read_raw(stream, u_channel);
			read_raw(stream, v_channel);

			auto executeTimeread = double(clock() - startTimeread) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Read Color Total Time read: " << std::fixed << std::setprecision(3) << executeTimeread
				<< " sec."
				<< std::endl;
			auto startTimesize = clock();

			cv::resize(u_channel, u_channel, size, 0, 0, cv::INTER_CUBIC);
			cv::resize(v_channel, v_channel, size, 0, 0, cv::INTER_CUBIC);

			auto executeTimesize = double(clock() - startTimesize) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Read Color Total Time Resize: " << std::fixed << std::setprecision(3) << executeTimesize
				<< " sec."
				<< std::endl;
			auto startTimemerge = clock();

			cv::Mat image(size, CV_MAKETYPE(cvdepth_from_bit_depth(bit_depth), 3));
			cv::Mat src[] = { y_channel, u_channel, v_channel };
			cv::merge(src, 3, image);

			auto executeTimemerge = double(clock() - startTimemerge) / CLOCKS_PER_SEC;
			std::cout
				<< std::endl
				<< "Read Color Total Time Merge: " << std::fixed << std::setprecision(3) << executeTimemerge
				<< " sec."
				<< std::endl;
			auto startTimenorm = clock();

			return image;
		}

		cv::Mat read_depth_YUV(std::string filepath, int frame, Parameters const& parameters) {
			auto size = parameters.getPaddedSize();
			auto bit_depth = parameters.getDepthBitDepth();
			cv::Mat image(size, CV_MAKETYPE(cvdepth_from_bit_depth(bit_depth), 1));
			std::ifstream stream(filepath, std::ios_base::binary);
			if (!stream.good()) {
				std::ostringstream what;
				what << "Failed to read raw YUV depth file \"" << filepath << "\"";
				throw std::runtime_error(what.str());
			}
			
			switch (parameters.getDepthColorFormat()) {
			case ColorFormat::YUV420:
				stream.seekg(size.area() * image.elemSize() * 3 / 2 * frame);
				break;
			case ColorFormat::YUV400:
				stream.seekg(size.area() * image.elemSize() * frame);
				break;
			default:
				throw std::logic_error("Unknown depth map color format");
			}

			read_raw(stream, image);
			return image;
		}

		cv::Mat read_color_RGB(std::string filepath, Parameters const& parameters) {
			cv::Mat image = cv::imread(filepath, cv::IMREAD_UNCHANGED);

			if (image.empty())
				throw std::runtime_error("Failed to read color file");
			if (image.size() != parameters.getPaddedSize())
			{
				//throw std::runtime_error("Color file does not have the expected size");
				std::cout << "Color file does not have the expected size" <<std::endl;
				cv::resize(image, image, parameters.getPaddedSize());
			}
			if (image.depth() != cvdepth_from_bit_depth(parameters.getColorBitDepth()))
			{
				std::cout << image.depth() << std::endl;
				throw std::runtime_error("Color file has wrong bit depth");
			}
			if (image.channels() != 3)
			{
				if (image.channels() == 4)
				{
					cv::Mat dst[4];
					cv::split(image, dst);
					cv::merge(dst, 3, image);
				}
				else
					throw std::runtime_error("Color file has wrong number of channels");
			}

			return image;
		}

		cv::Mat read_depth_RGB(std::string filepath, Parameters const& parameters) {
			cv::Mat image = cv::imread(filepath, cv::IMREAD_UNCHANGED);

			if (image.empty())
				throw std::runtime_error("Failed to read depth file");
			if (image.size() != parameters.getPaddedSize())
				//throw std::runtime_error("Depth file does not have the expected size");
				cv::resize(image, image, parameters.getPaddedSize(),0.0,0.0,cv::INTER_NEAREST);
			//if (image.depth() != cvdepth_from_bit_depth(parameters.getDepthBitDepth()))
			//	throw std::runtime_error("Depth file has the wrong bit depth");
			if (image.channels() != 1)
			{
				std::cout << "Depth file has the wrong number of channels" <<std::endl;
				cv::Mat dst[3];
				cv::split(image, dst);
				image = dst[2];
			}
			return image;
		}
	}

	int cvdepth_from_bit_depth(int bit_depth)
	{
		if (bit_depth >= 1 && bit_depth <= 8)
			return CV_8U;
		else if (bit_depth >= 9 && bit_depth <= 16)
			return CV_16U;
		else if (bit_depth == 32)
			return CV_32F;
		else throw std::invalid_argument("invalid raw image bit depth");
	}

	unsigned max_level(int bit_depth)
	{
		assert(bit_depth > 0 && bit_depth <= 16);
		return (1u << bit_depth) - 1u;
	}

	cv::Mat3f read_color(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimecolor = clock();
		// Load the image
		cv::Mat image;
		ColorSpace color_space;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") {
			image = read_color_RGB(filepath, parameters);//(filepath, frame, parameters);
			color_space = ColorSpace::YUV;
		}
		else if (frame == 0) {
			image = read_color_RGB(filepath, parameters);
			color_space = ColorSpace::RGB;
			std::cout << "RGBBBBBBBBB" << std::endl;
		}
		else {
			throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
			std::cout << "????????????" << std::endl;
		}

		auto executeTimecolor = double(clock() - startTimecolor) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color: " << std::fixed << std::setprecision(3) << executeTimecolor
			<< " sec."
			<< std::endl;
		auto startTimecrop = clock();

		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimecrop = double(clock() - startTimecrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop: " << std::fixed << std::setprecision(3) << executeTimecrop
			<< " sec."
			<< std::endl;
		auto startTimenorm = clock();

		// Normalize to [0, 1]
		cv::Mat3f color;
		
		if (image.depth() == CV_32F) {
			color = image;
		}
		else {
			
			image.convertTo(color, CV_32F, 1. / max_level(parameters.getColorBitDepth()));
		}
		auto executeTimenorm = double(clock() - startTimenorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time normalize : " << std::fixed << std::setprecision(3) << executeTimenorm
			<< " sec."
			<< std::endl;
		auto startTimeconv = clock();
		// Color space conversion
		if (color_space == ColorSpace::YUV && g_color_space == ColorSpace::RGB) {
			cv::cvtColor(color, color, cv::COLOR_YUV2BGR,3);
		}
		else if (color_space == ColorSpace::RGB && g_color_space == ColorSpace::YUV) {
			cv::cvtColor(color, color, cv::COLOR_BGR2YUV);
		}
		auto executeTimeconv = double(clock() - startTimeconv) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color conv: " << std::fixed << std::setprecision(3) << executeTimeconv
			<< " sec."
			<< std::endl;

		return color;
	}

	cv::Mat3f read_color_lucid(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimecolor = clock();
		// Load the image
		cv::Mat image;
		image.create(480, 640, CV_8UC3);
		ColorSpace color_space;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") { //enters this if
			std::cout << "input cam: " << inputCam << std::endl;
			image = getRgbDataLucid();

			if (image.empty()) { //check if image acquisition worked
				throw std::runtime_error("RGB image is empty");
			}

			//image = read_color_RGB(filepath, parameters);//(filepath, frame, parameters);
			color_space = ColorSpace::RGB;//YUV;
		}
		else if (frame == 0) {
			color_space = ColorSpace::RGB;
			image = read_color_RGB(filepath, parameters);
		}
		else {
			throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
		}

		auto executeTimecolor = double(clock() - startTimecolor) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color: " << std::fixed << std::setprecision(3) << executeTimecolor
			<< " sec."
			<< std::endl;
		auto startTimecrop = clock();

		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			std::cout << "cropping..." << std::endl;
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimecrop = double(clock() - startTimecrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop: " << std::fixed << std::setprecision(3) << executeTimecrop
			<< " sec."
			<< std::endl;

		auto startTimenorm = clock();

		// Normalize to [0, 1]
		cv::Mat3f color;
		std::cout << image.depth() << std::endl;
		if (image.depth() == CV_32F) {
			color = image;
		}
		else { //enters this else
			image.convertTo(color, CV_32F, 1. / max_level(parameters.getColorBitDepth()));
		}

		auto executeTimenorm = double(clock() - startTimenorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time normalize : " << std::fixed << std::setprecision(3) << executeTimenorm
			<< " sec."
			<< std::endl;

		auto startTimeconv = clock();
		// Color space conversion
		if (color_space == ColorSpace::YUV && g_color_space == ColorSpace::RGB) {
			std::cout << "yuv conv" << std::endl;
			cv::cvtColor(color, color, cv::COLOR_YUV2BGR, 3);
		}
		else if (color_space == ColorSpace::RGB && g_color_space == ColorSpace::YUV) { //enters this if
			std::cout << "RGB conv" << std::endl;
			//cv::cvtColor(color, color, cv::COLOR_BGR2YUV);

		}
		auto executeTimeconv = double(clock() - startTimeconv) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color conv: " << std::fixed << std::setprecision(3) << executeTimeconv
			<< " sec."
			<< std::endl;
		return color;
	}

	cv::Mat3f read_color_kinect(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimecolor = clock();
		// Load the image
		cv::Mat image;
		image.create(480, 640, CV_8UC3);
		ColorSpace color_space;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") {			
			//this is the path						
			std::cout << "input cam: " << inputCam << std::endl;
			image = getRgbData();												

			//image = read_color_RGB(filepath, parameters);//(filepath, frame, parameters);
			color_space = ColorSpace::RGB;//YUV;
		}
		else if (frame == 0) {			
			color_space = ColorSpace::RGB;
			image = read_color_RGB(filepath, parameters);
		}
		else {
			throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
		}
		
		auto executeTimecolor = double(clock() - startTimecolor) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color: " << std::fixed << std::setprecision(3) << executeTimecolor
			<< " sec."
			<< std::endl;
		auto startTimecrop = clock();

		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			std::cout << "cropping..." << std::endl;
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimecrop = double(clock() - startTimecrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop: " << std::fixed << std::setprecision(3) << executeTimecrop
			<< " sec."
			<< std::endl;

		auto startTimenorm = clock();

		// Normalize to [0, 1]
		cv::Mat3f color;
		if (image.depth() == CV_32F) {
			color = image;
		}
		else {
			image.convertTo(color, CV_32F, 1. / max_level(parameters.getColorBitDepth()));
		}

		auto executeTimenorm = double(clock() - startTimenorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time normalize : " << std::fixed << std::setprecision(3) << executeTimenorm
			<< " sec."
			<< std::endl;
		
		auto startTimeconv = clock();
		// Color space conversion
		if (color_space == ColorSpace::YUV && g_color_space == ColorSpace::RGB) {
			std::cout << "yuv conv" << std::endl;
			cv::cvtColor(color, color, cv::COLOR_YUV2BGR, 3);
		}
		else if (color_space == ColorSpace::RGB && g_color_space == ColorSpace::YUV) {
			std::cout << "RGB conv" << std::endl;
			//cv::cvtColor(color, color, cv::COLOR_BGR2YUV);
			
		}
		auto executeTimeconv = double(clock() - startTimeconv) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time color conv: " << std::fixed << std::setprecision(3) << executeTimeconv
			<< " sec."
			<< std::endl;
		return color;
	}

	cv::Mat1f read_depth_lucid(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimedepth = clock();
		// Load the image
		//cv::Mat image(parameters.getSize(),cv::IMREAD_UNCHANGED);
		cv::Mat image;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") {
			//image = read_depth_YUV(filepath, frame, parameters);
			image = getDepthDataLucid();
		}
		else if (frame == 0) {
			image = getDepthDataLucid();
		}
		else {
			//throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
			std::cout << "Reading multiple frames not (yet) supported for image files. Reading first frame" << std::endl;
			//image = read_depth_RGB(filepath, parameters);
		}
		auto executeTimedepth = double(clock() - startTimedepth) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth : " << std::fixed << std::setprecision(3) << executeTimedepth
			<< " sec."
			<< std::endl;

		auto startTimedcrop = clock();
		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimedcrop = double(clock() - startTimedcrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop depth: " << std::fixed << std::setprecision(3) << executeTimedcrop
			<< " sec."
			<< std::endl;

		auto startTimednorm = clock();
		// Do not manipulate floating-point depth maps (e.g. OpenEXR)
		if (image.depth() == CV_32F) {
			return image;
		}

		// Normalize to [0, 1]
		cv::Mat1f depth;
		image.convertTo(depth, CV_32F, 1. / max_level(parameters.getDepthBitDepth()));
		auto executeTimednorm = double(clock() - startTimednorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth normalize : " << std::fixed << std::setprecision(3) << executeTimednorm
			<< " sec."
			<< std::endl;
		auto startTimegl = clock();

		// 1000 is for 'infinitly far'
		auto neark = parameters.getDepthRange()[0];
		auto fark = parameters.getDepthRange()[1];
		if (fark >= 1000.f) {
			depth = neark / depth;
		}
		else {
			depth = fark * neark / (neark + depth * (fark - neark));
		}

		if (parameters.hasInvalidDepth()) {
			// Level 0 is for 'invalid'
			// Mark invalid pixels as NaN
			auto const NaN = std::numeric_limits<float>::quiet_NaN();
			depth.setTo(NaN, image == 0);
		}

		return depth;
	}

	/*
	cv::Mat1f read_depth(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimedepth = clock();
		// Load the image
		cv::Mat image;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") {
			image = read_depth_YUV(filepath, frame, parameters);
		}
		else if (frame == 0) {
			image = read_depth_RGB(filepath, parameters);
		}
		else {
			//throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
			std::cout << "Reading multiple frames not (yet) supported for image files. Reading first frame" << std::endl;
			image = read_depth_RGB(filepath, parameters);
		}
		auto executeTimedepth = double(clock() - startTimedepth) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth : " << std::fixed << std::setprecision(3) << executeTimedepth
			<< " sec."
			<< std::endl;

		auto startTimedcrop = clock();
		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimedcrop = double(clock() - startTimedcrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop depth: " << std::fixed << std::setprecision(3) << executeTimedcrop
			<< " sec."
			<< std::endl;

		auto startTimednorm = clock();
		// Do not manipulate floating-point depth maps (e.g. OpenEXR)
		if (image.depth() == CV_32F) {
			return image;
		}

		// Normalize to [0, 1]
		cv::Mat1f depth;
		image.convertTo(depth, CV_32F, 1. / max_level(parameters.getDepthBitDepth()));
		auto executeTimednorm = double(clock() - startTimednorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth normalize : " << std::fixed << std::setprecision(3) << executeTimednorm
			<< " sec."
			<< std::endl;
		auto startTimegl = clock();

		// 1000 is for 'infinitly far'
		auto near = parameters.getDepthRange()[0];
		auto far = parameters.getDepthRange()[1];
		if (far >= 1000.f) {
			depth = near / depth;
		}
		else {
			depth = far * near / (near + depth * (far - near));
		}

		if (parameters.hasInvalidDepth()) {
			// Level 0 is for 'invalid'
			// Mark invalid pixels as NaN
			auto const NaN = std::numeric_limits<float>::quiet_NaN();
			depth.setTo(NaN, image == 0);
		}

		return depth;
	}
	*/
	cv::Mat1f read_depth_kinect(std::string filepath, int frame, Parameters const& parameters)
	{
		auto startTimedepth = clock();
		// Load the image
		//cv::Mat image(parameters.getSize(),cv::IMREAD_UNCHANGED);
		cv::Mat image;
		if (filepath.substr(filepath.size() - 4, 4) == ".yuv") {
			//image = read_depth_YUV(filepath, frame, parameters);
			image = getDepthData();
		}
		else if (frame == 0) {
			image = getDepthData();
		}
		else {
			//throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
			std::cout << "Reading multiple frames not (yet) supported for image files. Reading first frame" << std::endl;
			//image = read_depth_RGB(filepath, parameters);
		}
		auto executeTimedepth = double(clock() - startTimedepth) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth : " << std::fixed << std::setprecision(3) << executeTimedepth
			<< " sec."
			<< std::endl;

		auto startTimedcrop = clock();
		// Crop out padded regions
		if (parameters.getPaddedSize() != parameters.getSize()) {
			image = image(parameters.getCropRegion()).clone();
		}
		auto executeTimedcrop = double(clock() - startTimedcrop) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time crop depth: " << std::fixed << std::setprecision(3) << executeTimedcrop
			<< " sec."
			<< std::endl;

		auto startTimednorm = clock();
		// Do not manipulate floating-point depth maps (e.g. OpenEXR)
		if (image.depth() == CV_32F) {
			return image;
		}

		// Normalize to [0, 1]
		cv::Mat1f depth;
		image.convertTo(depth, CV_32F, 1. / max_level(parameters.getDepthBitDepth()));
		auto executeTimednorm = double(clock() - startTimednorm) / CLOCKS_PER_SEC;
		std::cout
			<< std::endl
			<< "Load Total Time depth normalize : " << std::fixed << std::setprecision(3) << executeTimednorm
			<< " sec."
			<< std::endl;
		auto startTimegl = clock();

		// 1000 is for 'infinitly far'
		auto neark = parameters.getDepthRange()[0];
		auto fark = parameters.getDepthRange()[1];
		if (fark >= 1000.f) {
			depth = neark / depth;
		}
		else {
			depth = fark * neark / (neark + depth * (fark - neark));
		}

		if (parameters.hasInvalidDepth()) {
			// Level 0 is for 'invalid'
			// Mark invalid pixels as NaN
			auto const NaN = std::numeric_limits<float>::quiet_NaN();
			depth.setTo(NaN, image == 0);
		}

		return depth;
	}

	PolynomialDepth read_polynomial_depth(std::string filepath, int frame, Parameters const& parameters)
	{
		// Load the image
		cv::Mat image;
		PolynomialDepth pd;
		std::string ext = filepath.substr(filepath.size() - 4, 4);
		if (frame == 0 && ext == ".exr") {
			std::array<cv::Mat1f,20> polynomial;
			for (int i = 0; i < 20; ++i) {
				std::stringstream ss;
				ss << i;
				size_t pos = filepath.find('*');
				std::string f = filepath.substr(0, pos) + ss.str() + filepath.substr(pos+1, filepath.size());
				cv::Mat1f p = cv::imread(f, cv::IMREAD_UNCHANGED);
			//	cv::GaussianBlur(p1,p1,cv::Size(11,11),3);
                polynomial[i] = p;
			}
			pd = PolynomialDepth(polynomial);
		}
		else if (ext == ".yuv") {
			std::array<cv::Mat1f, 20> polynomial;
			for (int i = 0; i < 20; ++i) {
				std::stringstream ss;
				ss << i;
				size_t pos = filepath.find('*');
				std::string f = filepath.substr(0, pos) + ss.str() + filepath.substr(pos + 1, filepath.size());
				cv::Mat depth = read_depth_YUV(f, frame, parameters);
				cv::Mat1f p;
				depth.convertTo(p, CV_32F, 1. / max_level(parameters.getDepthBitDepth()));
				if (i != 9 && i != 19)
					p = (parameters.getMultiDepthRange()[1] - parameters.getMultiDepthRange()[0]) * p + parameters.getMultiDepthRange()[0];
				else if (i == 9)
					p = (parameters.getDepthRange()[0] + (parameters.getDepthRange()[1] - parameters.getDepthRange()[0]) * p) * parameters.getFocal()[0] / (parameters.getDepthRange()[1] * parameters.getDepthRange()[0]);
				polynomial[i] = p;
			}
			pd = PolynomialDepth(polynomial);
		}
		else {
			throw std::runtime_error("Readig multiple frames not (yet) supported for image files");
		}

		return pd;
	}
}
