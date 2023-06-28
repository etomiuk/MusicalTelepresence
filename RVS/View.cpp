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

#include "View.hpp"
#include "inpainting.hpp"
#include "image_loading.hpp"

#include <fstream>
#include <iostream>

#include <cmath>
#include <cstdio>

#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

/*
// OpenGL Variables
const int width = 640;
const int height = 480;
long depthToRgbMap[width * height * 2];
// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;

// Kinect variables
HANDLE depthStream;
HANDLE rgbStream;
INuiSensor* sensor;
*/

namespace rvs
{
	// Initialize all maps at once
	View::View(cv::Mat3f color, cv::Mat1f depth, cv::Mat1f quality, cv::Mat1f validity)
	{
		assign(color, depth, quality, validity);
	}

	View::~View() {}

	// Initialize all maps at once
	void View::assign(cv::Mat3f color, cv::Mat1f depth, cv::Mat1f quality, cv::Mat1f validity)
	{
		m_color = color;
		m_depth = depth;
		m_quality = quality;
		m_validity = validity;
		validate();
	}
	void View::assign(cv::Mat3f color, cv::Mat1f depth, cv::Mat1f quality, cv::Mat1f validity, PolynomialDepth polynomial_depth)
	{
		m_color = color;
		m_depth = depth;
		m_quality = quality;
		m_validity = validity;
		m_polynomial_depth = polynomial_depth;
		validate();
	}

	// Return the texture
	cv::Mat3f View::get_color() const
	{
		validate();
		CV_Assert(!m_color.empty());
		return m_color;
	}

	// Return the depth map (same size as texture)
	cv::Mat1f View::get_depth() const
	{
		validate();
		CV_Assert(!m_depth.empty());
		return m_depth;
	}

	PolynomialDepth View::get_polynomial_depth() const
	{
		validate();
		return m_polynomial_depth;
	}

	DisplacementMethod InputView::get_displacementMethod() const
	{
		return parameters.getDisplacementMethod();
	}

	// Return the quality map (same size as texture)
	cv::Mat1f View::get_quality() const
	{
		validate();
		CV_Assert(!m_quality.empty());
		return m_quality;
	}

	// Return the validity map (same size as texture)
	cv::Mat1f View::get_validity() const
	{
		validate();
		CV_Assert(!m_validity.empty());
		return m_validity;
	}

	// Return the size of the texture and depth map
	cv::Size View::get_size() const
	{
		validate();
		return m_color.size();
	}

	// Return a mask with all valid depth values
	cv::Mat1b View::get_depth_mask() const
	{
		return get_depth() > 0.f; // excludes NaN's
	}

	// Calculate a mask for inpainting
	cv::Mat1b View::get_inpaint_mask() const
	{
		auto inpaint_mask = cv::Mat1b(get_size(), 255);
		inpaint_mask.setTo(0, get_quality() > 0.f); // excludes NaN's
		return inpaint_mask;
	}

	// Calculate a mask with valid pixels for masked output
	cv::Mat1b View::get_validity_mask(float threshold) const
	{
		auto validity_mask = cv::Mat1b(get_size(), 255);
		validity_mask.setTo(0, get_validity() > threshold); // excludes NaN's
		return validity_mask;
	}

	void View::validate() const
	{
		auto size = m_color.size();
		CV_Assert(m_depth.empty() || m_depth.size() == size);
		CV_Assert(m_quality.empty() || m_quality.size() == size);
		CV_Assert(m_validity.size() == m_quality.size());
	}

	// Load a color image and depth map
	InputView::InputView(std::string const& filepath_color, std::string const& filepath_depth, int frame, Parameters const& parameters)
		:
		filepath_color(filepath_color),
		filepath_depth(filepath_depth),
		frame(frame),
		parameters(parameters)
	{
		load();
	}

	// Load a color image and depth map
	void InputView::load()
	{
		if (parameters.getDisplacementMethod() == DisplacementMethod::depth) { //this is thwe one that enters
			cv::Mat3f a = read_color_lucid(filepath_color, frame, parameters);				//changed from read_color to read_color_kinect --> changed to lucid
			assign(
				a,
				read_depth_lucid(filepath_depth, frame, parameters),					//changed to kinect depth --> then changed to lucid
				cv::Mat1f(),
				cv::Mat1f());
		}
		else
		{
		        if (parameters.getDisplacementMethod()==DisplacementMethod::polynomial){
                        assign(
							read_color_lucid(filepath_color, frame, parameters),
                            cv::Mat1f::zeros(parameters.getSize()),
                            cv::Mat1f(),
                            cv::Mat1f(),
                            read_polynomial_depth(filepath_depth, frame, parameters));
			}
		}
		loaded = true;
	}

	void InputView::unload()
	{
		assign(cv::Mat3f(), cv::Mat1f(), cv::Mat1f(), cv::Mat1f());
		loaded = false;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Kinect functions
	///////////////////////////////////////////////////////////////////////////////////////////////
	/*
	
	//initialize the kinect camera
	bool initKinect() {
		// Get a working kinect sensor
		int numSensors;
		if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
		if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

		// Initialize sensor
		sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,        // Image stream flags, e.g. near mode
			2,        // Number of frames to buffer
			NULL,     // Event handle
			&depthStream);
		sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
			NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
			0,      // Image stream flags, e.g. near mode
			2,      // Number of frames to buffer
			NULL,   // Event handle
			&rgbStream);
		return sensor;
	}

	//Get Detph image 
	void getDepthData(GLubyte* dest) {
		float* fdest = (float*)dest;
		long* depth2rgb = (long*)depthToRgbMap;
		NUI_IMAGE_FRAME imageFrame;
		NUI_LOCKED_RECT LockedRect;
		if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
		INuiFrameTexture* texture = imageFrame.pFrameTexture;
		texture->LockRect(0, &LockedRect, NULL, 0);
		if (LockedRect.Pitch != 0) {
			const USHORT* curr = (const USHORT*)LockedRect.pBits;
			for (int j = 0; j < height; ++j) {
				for (int i = 0; i < width; ++i) {
					// Get depth of pixel in millimeters
					USHORT depth = NuiDepthPixelToDepth(*curr++);
					// Store coordinates of the point corresponding to this pixel
					Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, depth << 3, NUI_IMAGE_RESOLUTION_640x480);
					*fdest++ = pos.x / pos.w;
					*fdest++ = pos.y / pos.w;
					*fdest++ = pos.z / pos.w;
					// Store the index into the color array corresponding to this pixel
					NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
						NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
						i, j, depth << 3, depth2rgb, depth2rgb + 1);
					depth2rgb += 2;
				}
			}
		}
		texture->UnlockRect(0);
		sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
	}

	// Get RGB Image 
	void getRgbData(GLubyte* dest) {
		float* fdest = (float*)dest;
		long* depth2rgb = (long*)depthToRgbMap;
		NUI_IMAGE_FRAME imageFrame;
		NUI_LOCKED_RECT LockedRect;
		if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
		INuiFrameTexture* texture = imageFrame.pFrameTexture;
		texture->LockRect(0, &LockedRect, NULL, 0);
		if (LockedRect.Pitch != 0) {
			const BYTE* start = (const BYTE*)LockedRect.pBits;
			for (int j = 0; j < height; ++j) {
				for (int i = 0; i < width; ++i) {
					// Determine rgb color for each depth pixel
					long x = *depth2rgb++;
					long y = *depth2rgb++;
					// If out of bounds, then don't color it at all
					if (x < 0 || y < 0 || x > width || y > height) {
						for (int n = 0; n < 3; ++n) *(fdest++) = 0.0f;
					}
					else {
						const BYTE* curr = start + (x + width * y) * 4;
						for (int n = 0; n < 3; ++n) *(fdest++) = curr[2 - n] / 255.0f;
					}

				}
			}
		}
		texture->UnlockRect(0);
		sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
	}

	//Call image extraction functions
	void getKinectData() {
		GLubyte* ptr;
		glBindBuffer(GL_ARRAY_BUFFER, vboId);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			getDepthData(ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, cboId);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			getRgbData(ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
	}
	*/


}
