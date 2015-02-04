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

#include "ModuleController.h"

#include <exception>
#include <sstream>
#include <string>

#include "SynthesizerController.h"
#include "Util.h"



ModuleController::ModuleController(std::istream& in, std::ostream& out, const char* configFilePath)
		: state(STATE_IDLE)
		, in_(in)
		, out_(out)
		, configFilePath_(configFilePath)
		, newCommand_(false)
		, commandType_(COMMAND_NONE)
		, synthController_(new SynthesizerController(*this))
{
}

ModuleController::~ModuleController()
{
}

void
ModuleController::exec()
{
	std::string line;
	while (std::getline(in_, line)) {
		if (line == "INIT") {
			handleInitCommand();
		} else if (line == "AUDIO") {
			handleAudioCommand();
		} else if (line == "LOGLEVEL") {
			handleLogLevelCommand();
		} else if (line == "SET") {
			handleSetCommand();
		} else if (line == "SPEAK" || line == "CHAR" || line == "KEY" || line == "SOUND_ICON") {
			handleSpeakCommand();
		} else if (line == "STOP") {
			handleStopCommand();
		} else if (line == "QUIT") {
			handleQuitCommand();
			return;
		} else {
			sendResponse("300 ERROR");
		}
	}
}

void
ModuleController::handleInitCommand()
{
	setSynthCommand(COMMAND_INIT, configFilePath_);
}

void
ModuleController::handleAudioCommand()
{
	sendResponse("207 OK RECEIVING AUDIO SETTINGS");

//	audio_output_method=alsa
//	audio_oss_device=/dev/dsp
//	audio_alsa_device=default
//	audio_nas_server=tcp/localhost:5450
//	audio_pulse_server=default
//	audio_pulse_min_length=100

	std::string line;
	while (std::getline(in_, line)) {
		if (line == ".") break;

		// Ignore data.
	}

	sendResponse("203 OK AUDIO INITIALIZED");
}

void
ModuleController::handleLogLevelCommand()
{
	sendResponse("207 OK RECEIVING LOGLEVEL SETTINGS");

	std::string line;
	while (std::getline(in_, line)) {
		if (line == ".") break;

		auto pair = Util::getNameAndValue(line);
		if (pair.first == "log_level") {
			config_.setLogLevel(pair.second);
		}
	}

	sendResponse("203 OK LOG LEVEL SET");
}

void
ModuleController::handleSetCommand()
{
	sendResponse("203 OK RECEIVING SETTINGS");

	{
		std::lock_guard<std::mutex> lock(configMutex_);

		std::string line;
		while (std::getline(in_, line)) {
			if (line == ".") break;

			auto p = Util::getNameAndValue(line);
			if (p.first == "pitch") {
				config_.setPitch(p.second);
			} else if (p.first == "rate") {
				config_.setRate(p.second);
			} else if (p.first == "volume") {
				config_.setVolume(p.second);
			} else if (p.first == "punctuation_mode") {
				config_.setPunctuationMode(p.second);
			} else if (p.first == "spelling_mode") {
				config_.setSpellingMode(p.second);
			} else if (p.first == "cap_let_recogn") {
				config_.setCapitalLetterRecognition(p.second);
			} else if (p.first == "voice") {
				config_.setVoice(p.second);
			} else if (p.first == "language") {
				config_.setLanguage(p.second);
			} else if (p.first == "synthesis_voice") {
				config_.setSynthesisVoice(p.second);
			}
		}
	}
	setSynthCommand(COMMAND_SET, std::string());

	sendResponse("203 OK SETTINGS RECEIVED");
}

void
ModuleController::handleSpeakCommand()
{
	sendResponse("202 OK RECEIVING MESSAGE");

	std::ostringstream out;
	std::string line;
	while (std::getline(in_, line)) {
		if (line == ".") break;
		if (out.tellp() != 0) {
			out << ' ';
		}
		out << line;
	}
	std::string msg = out.str();

	Util::stripSSML(msg);

	state = STATE_SPEAKING;
	setSynthCommand(COMMAND_SPEAK, msg);
}

void
ModuleController::handleStopCommand()
{
	state = STATE_STOP_REQUESTED;
}

void
ModuleController::handleQuitCommand()
{
	setSynthCommand(COMMAND_QUIT, std::string());

	synthController_->wait();

	sendResponse("210 OK QUIT");
}

void
ModuleController::setSynthCommand(CommandType type, const std::string& message)
{
	{
		std::unique_lock<std::mutex> lock(commandMutex_);

		while (newCommand_) {
			commandReceivedCondition_.wait(lock);
		}

		commandType_ = type;
		commandMessage_ = message;

		newCommand_ = true;
	}
	commandSentCondition_.notify_one();
}

void
ModuleController::getSynthCommand(ModuleController::CommandType& type, std::string& message, bool wait)
{
	{
		std::unique_lock<std::mutex> lock(commandMutex_);

		if (wait) {
			while (!newCommand_) {
				commandSentCondition_.wait(lock);
			}
		} else {
			if (!newCommand_) {
				type = COMMAND_NONE;
				message.clear();
				return;
			}
		}

		type = commandType_;
		message = commandMessage_;

		newCommand_ = false;
	}
	commandReceivedCondition_.notify_one();
}

void
ModuleController::setSynthCommandResult(CommandType type, bool failed, const std::string& msg)
{
	switch (type) {
	case COMMAND_INIT:
		if (failed) {
			std::ostringstream stream;
			stream <<
				"399-Gnuspeech: " << msg << '\n' <<
				"399 ERR CANT INIT MODULE";
			sendResponse(stream.str().c_str());
		} else {
			sendResponse(
				"299-Gnuspeech: Initialized successfully.\n"
				"299 OK LOADED SUCCESSFULLY");
		}
		break;
	case COMMAND_SPEAK:
		if (failed) {
			sendResponse("301 ERROR CANT SPEAK");
			state = STATE_IDLE;
		} else {
			sendResponse("200 OK SPEAKING");
		}
		break;
	default:
		break;
	}
}

void
ModuleController::sendResponse(const char* msg)
{
	std::lock_guard<std::mutex> lock(responseMutex_);
	out_ << msg << std::endl;
}

void
ModuleController::sendBeginEvent()
{
	std::lock_guard<std::mutex> lock(responseMutex_);
	out_ << "701 BEGIN" << std::endl;
}

void
ModuleController::sendEndEvent()
{
	responseMutex_.lock();
	out_ << "702 END" << std::endl;
	responseMutex_.unlock();

	state = STATE_IDLE;
}

void
ModuleController::sendStopEvent()
{
	responseMutex_.lock();
	out_ << "703 STOP" << std::endl;
	responseMutex_.unlock();

	state = STATE_IDLE;
}

void
ModuleController::getConfigCopy(ModuleConfiguration& config) {
	std::lock_guard<std::mutex> lock(configMutex_);
	config = config_;
}
