/***************************************************************************
 *  Copyright 2015 Marcelo Y. Matuda                                       *
 *                                                                         *
 *  This program is free software: you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           *
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

#include "SynthesizerController.h"

#include <cmath> /* pow */
#include <exception>
#include <iostream>
#include <cstring>
#include <string>

#include "Controller.h"
#include "en/phonetic_string_parser/PhoneticStringParser.h"
#include "en/text_parser/TextParser.h"
#include "KeyValueFileReader.h"
#include "Model.h"
#include "Tube.h"
#include "Util.h"

#define TRM_CONTROL_MODEL_CONFIG_FILE "/monet.xml"



SynthesizerController::SynthesizerController(ModuleController& moduleController)
		: moduleController_(moduleController)
		, synthThread_(&SynthesizerController::exec, std::ref(*this))
		, defaultPitchOffset_(0.0)
		, audioBufferIndex_(0)
		, numInputChannels_(0)
		, audioOutputDeviceIndex_(-1)
{
}

SynthesizerController::~SynthesizerController()
{
}

void
SynthesizerController::exec()
{
	for (;;) {
		moduleController_.getSynthCommand(commandType_, commandMessage_, true);

		switch (commandType_) {
		case ModuleController::COMMAND_INIT:
			init();
			break;
		case ModuleController::COMMAND_SET:
			set();
			break;
		case ModuleController::COMMAND_SPEAK:
			speak();
			break;
		case ModuleController::COMMAND_QUIT:
			return;
		default:
			break;
		}
	}
}

void
SynthesizerController::wait()
{
	if (synthThread_.joinable()) {
		synthThread_.join();
	}
}

void
SynthesizerController::init()
{
	try {
		GS::KeyValueFileReader reader(commandMessage_);
		std::string configDirPath = reader.value<std::string>("config_dir_path");
		audioOutputDeviceIndex_ = reader.value<int>("audio_output_device_index");

		model_.reset(new GS::TRMControlModel::Model());
		model_->load(configDirPath.c_str(), TRM_CONTROL_MODEL_CONFIG_FILE);

		modelController_.reset(new GS::TRMControlModel::Controller(configDirPath.c_str(), *model_));
		GS::TRMControlModel::Configuration& trmModelConfig = modelController_->trmControlModelConfig();
		defaultPitchOffset_ = trmModelConfig.pitchOffset;

		textParser_.reset(new GS::En::TextParser(configDirPath.c_str()));
		phoneticStringParser_.reset(new GS::En::PhoneticStringParser(configDirPath.c_str(), *modelController_));

		tube_.reset(new GS::TRM::Tube);
	} catch (const std::exception& exc) {
		std::ostringstream msg;
		msg << "[SynthesizerController::init] Could not initialize the synthesis controller. Reason: " << exc.what() << '.';
		std::string s = msg.str();
		std::cerr << s << std::endl;
		Util::removeLF(s);
		moduleController_.setSynthCommandResult(commandType_, true, s);
		return;
	}

	moduleController_.setSynthCommandResult(commandType_, false, std::string());
}

void
SynthesizerController::set()
{
	moduleController_.getConfigCopy(moduleConfig_);

	GS::TRMControlModel::Configuration& trmModelConfig = modelController_->trmControlModelConfig();
	trmModelConfig.pitchOffset = defaultPitchOffset_ + moduleConfig_.pitch / 5.0;
	trmModelConfig.tempo = std::pow(10.0, moduleConfig_.rate / 100.0);

	GS::TRM::Configuration& trmConfig = modelController_->trmConfig();
	trmConfig.volume = (moduleConfig_.volume < 0.0) ? (60.0 * (1.0 + moduleConfig_.volume / 100.0)) : 60.0;
}

void
SynthesizerController::speak()
{
	if (!modelController_) {
		moduleController_.setSynthCommandResult(commandType_, true, "Not initialized.");
		return;
	}

	//--------------------
	// Generate the audio.

	try {
		// Reset the stream.
		trmParamStream_.str("");
		trmParamStream_.clear();

		std::string phoneticString = textParser_->parseText(commandMessage_.c_str());
		modelController_->synthesizePhoneticString(*phoneticStringParser_, phoneticString.c_str(), trmParamStream_);
		tube_->synthesizeToBuffer(trmParamStream_, audioBuffer_);
	} catch (const std::exception& exc) {
		std::ostringstream msg;
		msg << "[SynthesizerController::speak] Could not synthesize the text. Reason: " << exc.what() << '.';
		std::string s = msg.str();
		std::cerr << s << std::endl;
		Util::removeLF(s);
		moduleController_.setSynthCommandResult(commandType_, true, s);
		return;
	}
	moduleController_.setSynthCommandResult(commandType_, false, std::string());

	//----------------
	// Play the audio.

	moduleController_.sendBeginEvent();

	double sampleRate = tube_->outputRate();
	numInputChannels_ = tube_->numChannels();
	audioBufferIndex_ = 0;

	try {
		portaudio::System& sys = portaudio::System::instance();
		if (audioOutputDeviceIndex_ >= sys.deviceCount()) {
			std::cerr << "Invalid audio output device index: " << audioOutputDeviceIndex_ << " (should be < " << sys.deviceCount() << "). Using default." << std::endl;
			audioOutputDeviceIndex_ = -1;
		}
		portaudio::Device& dev = (audioOutputDeviceIndex_ != -1) ?
						sys.deviceByIndex(audioOutputDeviceIndex_) :
						sys.defaultOutputDevice();
		portaudio::DirectionSpecificStreamParameters outParams(dev, 2 /* channels */, portaudio::FLOAT32,
							true /* interleaved */, dev.defaultLowOutputLatency(), nullptr);
		portaudio::StreamParameters params(portaudio::DirectionSpecificStreamParameters::null(), outParams, sampleRate,
							paFramesPerBufferUnspecified, paClipOff);
		portaudio::MemFunCallbackStream<SynthesizerController> stream(params, *this, &SynthesizerController::portaudioCallback);

		stream.start();
		while (stream.isActive()) {
			sys.sleep(100 /* ms */);

			unsigned int moduleControllerState = moduleController_.state;
			if (moduleControllerState == ModuleController::STATE_STOP_REQUESTED) {
				stream.stop();
				moduleController_.sendStopEvent();
				return;
			}
		}
		stream.stop();

	} catch (const std::exception& exc) {
		std::cerr << "[SynthesizerController::speak][PortAudio] Caught exception: " << exc.what() << '.' << std::endl;
	}

	moduleController_.sendEndEvent();
}

int
SynthesizerController::portaudioCallback(const void* /*inputBuffer*/, void* paOutputBuffer, unsigned long framesPerBuffer,
			const PaStreamCallbackTimeInfo* /*timeInfo*/, PaStreamCallbackFlags /*statusFlags*/)
{
	float* out = static_cast<float*>(paOutputBuffer);
	unsigned int numFrames = 0;
	if (numInputChannels_ == 2) {
		unsigned int framesAvailable = (audioBuffer_.size() - audioBufferIndex_) / 2;
		numFrames = (framesAvailable > framesPerBuffer) ? framesPerBuffer : framesAvailable;
		for (unsigned int i = 0; i < numFrames; ++i) {
			unsigned int baseIndex = i * 2;
			float* src = &audioBuffer_[audioBufferIndex_ + baseIndex];
			out[baseIndex]     = *src;
			out[baseIndex + 1] = *(src + 1);
		}
		audioBufferIndex_ += numFrames * 2;
	} else { // 1 input channel
		unsigned int framesAvailable = audioBuffer_.size() - audioBufferIndex_;
		numFrames = (framesAvailable > framesPerBuffer) ? framesPerBuffer : framesAvailable;
		for (unsigned int i = 0; i < numFrames; ++i) {
			unsigned int baseIndex = i * 2;
			float* src = &audioBuffer_[audioBufferIndex_ + i];
			out[baseIndex]     = *src;
			out[baseIndex + 1] = *src;
		}
		audioBufferIndex_ += numFrames;
	}
	for (unsigned int i = numFrames; i < framesPerBuffer; ++i) {
		unsigned int baseIndex = i * 2;
		out[baseIndex]     = 0;
		out[baseIndex + 1] = 0;
	}
	if (audioBufferIndex_ == audioBuffer_.size()) {
		return paComplete;
	} else {
		return paContinue;
	}
}
