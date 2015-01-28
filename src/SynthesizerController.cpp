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
#include <cstring>
#include <string>

#include "portaudiocpp/PortAudioCpp.hxx"

#include "Controller.h"
#include "en/phonetic_string_parser/PhoneticStringParser.h"
#include "en/text_parser/TextParser.h"
#include "KeyValueFileReader.h"
#include "Model.h"
#include "Tube.h"
#include "Util.h"

#define TRM_CONTROL_MODEL_CONFIG_FILE "/monet.xml"
#define AUDIO_BLOCK_SIZE 8192
#define PORTAUDIO_FRAMES_PER_BUFFER 2048
//#define PORTAUDIO_FRAMES_PER_BUFFER paFramesPerBufferUnspecified



SynthesizerController::SynthesizerController(ModuleController& moduleController)
		: moduleController_(moduleController)
		, synthThread_(&SynthesizerController::exec, std::ref(*this))
		, defaultPitchOffset_(0.0)
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
		tube_->synthesizeToBuffer(trmParamStream_, outputBuffer_);
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
	unsigned int numChannels = tube_->numChannels();

	try {
		portaudio::System& sys = portaudio::System::instance();
		portaudio::Device& dev = sys.defaultOutputDevice();
		portaudio::DirectionSpecificStreamParameters outParams(dev, numChannels, portaudio::FLOAT32,
							true /* interleaved */, dev.defaultHighOutputLatency(), nullptr);
		portaudio::StreamParameters params(portaudio::DirectionSpecificStreamParameters::null(), outParams, sampleRate,
							PORTAUDIO_FRAMES_PER_BUFFER, paClipOff);
		portaudio::BlockingStream stream(params);
		stream.start();

		for (unsigned int frame = 0, totalNumFrames = outputBuffer_.size() / numChannels; frame < totalNumFrames; frame += AUDIO_BLOCK_SIZE) {

			unsigned int endFrame = frame + AUDIO_BLOCK_SIZE;
			if (endFrame > totalNumFrames) {
				endFrame = totalNumFrames;
			}

			stream.write(&outputBuffer_[frame * numChannels], endFrame - frame);

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
