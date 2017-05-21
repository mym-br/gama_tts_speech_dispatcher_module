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

#include "ConfigurationData.h"
#include "Controller.h"
#include "Model.h"
#include "TextParser.h"
#include "Util.h"
#include "VocalTractModel.h"

#define VTM_CONTROL_MODEL_CONFIG_FILE "/artic.xml"
#define VTM_CONFIG_FILE_NAME "/vtm.config"
#define FADE_OUT_TIME_MS 30.0



SynthesizerController::SynthesizerController(ModuleController& moduleController)
		: moduleController_(moduleController)
		, synthThread_(&SynthesizerController::exec, std::ref(*this))
		, defaultPitchOffset_(0.0)
		, audioBufferIndex_(0)
		, numInputChannels_(0)
		, audioOutputDeviceIndex_(-1)
		, state_(STATE_STOPPED)
		, fadeOutAmplitude_(1.0)
		, fadeOutDelta_(0.0)
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
		GS::ConfigurationData data(commandMessage_);
		const std::string configDirPath = data.value<std::string>("config_dir_path");
		const std::string voiceFile = data.value<std::string>("voice_file");
		audioOutputDeviceIndex_ = data.value<int>("audio_output_device_index");

		model_ = std::make_unique<GS::VTMControlModel::Model>();
		model_->load(configDirPath.c_str(), VTM_CONTROL_MODEL_CONFIG_FILE);

		modelController_ = std::make_unique<GS::VTMControlModel::Controller>(configDirPath.c_str(), *model_);
		const GS::VTMControlModel::Configuration& vtmControlConfig = modelController_->vtmControlModelConfiguration();
		defaultPitchOffset_ = vtmControlConfig.pitchOffset;

		textParser_ = GS::VTMControlModel::TextParser::getInstance(configDirPath.c_str(), vtmControlConfig);

		std::ostringstream vtmConfigFilePath;
		vtmConfigFilePath << configDirPath << VTM_CONFIG_FILE_NAME;

		std::ostringstream voiceFilePath;
		voiceFilePath << configDirPath << '/' << voiceFile;

		GS::ConfigurationData vtmData(voiceFilePath.str());
		vtmData.insert(GS::ConfigurationData(vtmConfigFilePath.str()));

		vocalTractModel_ = GS::VTM::VocalTractModel::getInstance(vtmData);
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

	GS::VTMControlModel::Configuration& vtmControlConfig = modelController_->vtmControlModelConfiguration();
	vtmControlConfig.pitchOffset = defaultPitchOffset_ + moduleConfig_.pitch / 5.0;
	vtmControlConfig.tempo = std::pow(10.0, moduleConfig_.rate / 100.0);
//	if (moduleConfig_.voice == "female1") {
//	} else if (moduleConfig_.voice == "male2") {
//	} else if (moduleConfig_.voice == "child_male") {
//	} else if (moduleConfig_.voice == "child_female") {
//	} else { // male1 and others
//	}
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
		vtmParamStream_.str("");
		vtmParamStream_.clear();

		std::string phoneticString = textParser_->parse(commandMessage_.c_str());
		modelController_->fillParameterStream(phoneticString.c_str(), vtmParamStream_);
		vocalTractModel_->synthesizeToBuffer(vtmParamStream_, audioBuffer_);
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

	const double sampleRate = vocalTractModel_->outputRate();
	audioBufferIndex_ = 0;

	fadeOutAmplitude_ = 1.0;
	fadeOutDelta_ = 1.0 / (FADE_OUT_TIME_MS * 1.0e-3 * sampleRate);

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

		state_ = STATE_PLAYING;
		bool stopping = false;
		stream.start();
		while (stream.isActive()) {
			sys.sleep(100 /* ms */);

			if (!stopping) {
				unsigned int moduleControllerState = moduleController_.state();
				if (moduleControllerState == ModuleController::STATE_STOP_REQUESTED) {
					state_ = STATE_STOPPING;
					stopping = true;
				}
			}
		}
		stream.stop();
		if (stopping) {
			moduleController_.sendStopEvent();
			return;
		}
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
	const unsigned int st = state_;

	const unsigned int framesAvailable = audioBuffer_.size() - audioBufferIndex_;
	const unsigned int numFrames = (framesAvailable > framesPerBuffer) ? framesPerBuffer : framesAvailable;
	if (st == STATE_STOPPING) {
		for (unsigned int i = 0; i < numFrames; ++i) {
			fadeOutAmplitude_ -= fadeOutDelta_;
			if (fadeOutAmplitude_ < 0.0) fadeOutAmplitude_ = 0.0;
			const unsigned int baseIndex = i * 2;
			const float value = audioBuffer_[audioBufferIndex_ + i] * fadeOutAmplitude_;
			out[baseIndex]     = value;
			out[baseIndex + 1] = value;
		}
	} else {
		for (unsigned int i = 0; i < numFrames; ++i) {
			const unsigned int baseIndex = i * 2;
			const float value = audioBuffer_[audioBufferIndex_ + i];
			out[baseIndex]     = value;
			out[baseIndex + 1] = value;
		}
	}
	audioBufferIndex_ += numFrames;

	for (unsigned int i = numFrames; i < framesPerBuffer; ++i) {
		const unsigned int baseIndex = i * 2;
		out[baseIndex]     = 0;
		out[baseIndex + 1] = 0;
	}
	if (st == STATE_STOPPING && fadeOutAmplitude_ == 0.0) {
		state_ = STATE_STOPPED;
		return paComplete;
	}
	if (audioBufferIndex_ == audioBuffer_.size()) {
		state_ = STATE_STOPPED;
		return paComplete;
	} else {
		return paContinue;
	}
}
