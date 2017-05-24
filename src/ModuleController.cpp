/***************************************************************************
 *  Copyright 2015, 2017 Marcelo Y. Matuda                                 *
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

#include <sstream>

#include "SynthesizerController.h"
#include "Util.h"



namespace {

const std::string     audioCmdStr{"AUDIO"};
const std::string      charCmdStr{"CHAR"};
const std::string      initCmdStr{"INIT"};
const std::string       keyCmdStr{"KEY"};
const std::string  logLevelCmdStr{"LOGLEVEL"};
const std::string      quitCmdStr{"QUIT"};
const std::string       setCmdStr{"SET"};
const std::string soundIconCmdStr{"SOUND_ICON"};
const std::string     speakCmdStr{"SPEAK"};
const std::string      stopCmdStr{"STOP"};

const std::string respAudioInitStr{      "207 OK RECEIVING AUDIO SETTINGS"};
const std::string respAudioDoneStr{      "203 OK AUDIO INITIALIZED"};
const std::string respBeginEventStr{     "701 BEGIN"};
const std::string respEndEventStr{       "702 END"};
const std::string respExecFailStr{       "300 ERROR"};
const std::string respLogLevelInitStr{   "207 OK RECEIVING LOGLEVEL SETTINGS"};
const std::string respLogLevelDoneStr{   "203 OK LOG LEVEL SET"};
const std::string respQuitDoneStr{       "210 OK QUIT"};
const std::string respSetInitStr{        "203 OK RECEIVING SETTINGS"};
const std::string respSetDoneStr{        "203 OK SETTINGS RECEIVED"};
const std::string respSpeakInitStr{      "202 OK RECEIVING MESSAGE"};
const std::string respStopEventStr{      "703 STOP"};
const std::string respSynthInitDoneStr{  "299-GamaTTS: Initialized successfully.\n"
                                         "299 OK LOADED SUCCESSFULLY"};
const std::string respSynthInitFailStr_1{"399-GamaTTS: "};
const std::string respSynthInitFailStr_2{"399 ERR CANT INIT MODULE"};
const std::string respSynthSpeakFailStr{ "301 ERROR CANT SPEAK"};
const std::string respSynthSpeakDoneStr{ "200 OK SPEAKING"};

const std::string    capLetRecognStr{"cap_let_recogn"};
const std::string             dotStr{"."};
const std::string        languageStr{"language"};
const std::string        logLevelStr{"log_level"};
const std::string           pitchStr{"pitch"};
const std::string punctuationModeStr{"punctuation_mode"};
const std::string            rateStr{"rate"};
const std::string    spellingModeStr{"spelling_mode"};
const std::string  synthesisVoiceStr{"synthesis_voice"};
const std::string           voiceStr{"voice"};
const std::string          volumeStr{"volume"};

}

ModuleController::ModuleController(std::istream& in, std::ostream& out, const char* configFilePath)
		: state_{STATE_IDLE}
		, in_{in}
		, out_{out}
		, configFilePath_{configFilePath}
		, newCommand_{}
		, commandType_{COMMAND_NONE}
		, synthController_{std::make_unique<SynthesizerController>(*this)}
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
		if (line == speakCmdStr || line == charCmdStr || line == keyCmdStr || line == soundIconCmdStr) {
			handleSpeakCommand();
		} else if (line == initCmdStr) {
			handleInitCommand();
		} else if (line == audioCmdStr) {
			handleAudioCommand();
		} else if (line == logLevelCmdStr) {
			handleLogLevelCommand();
		} else if (line == setCmdStr) {
			handleSetCommand();
		} else if (line == stopCmdStr) {
			handleStopCommand();
		} else if (line == quitCmdStr) {
			handleQuitCommand();
			return;
		} else {
			sendResponse(respExecFailStr);
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
	sendResponse(respAudioInitStr);

//	audio_output_method=alsa
//	audio_oss_device=/dev/dsp
//	audio_alsa_device=default
//	audio_nas_server=tcp/localhost:5450
//	audio_pulse_server=default
//	audio_pulse_min_length=100

	std::string line;
	while (std::getline(in_, line)) {
		if (line == dotStr) break;

		// Ignore data.
	}

	sendResponse(respAudioDoneStr);
}

void
ModuleController::handleLogLevelCommand()
{
	sendResponse(respLogLevelInitStr);

	std::string line;
	while (std::getline(in_, line)) {
		if (line == dotStr) break;

		auto p = Util::getNameAndValue(line);
		if (p.first == logLevelStr) {
			std::lock_guard<std::mutex> lock(configMutex_);
			config_.setLogLevel(p.second);
		}
	}

	sendResponse(respLogLevelDoneStr);
}

void
ModuleController::handleSetCommand()
{
	sendResponse(respSetInitStr);

	{
		std::lock_guard<std::mutex> lock(configMutex_);

		std::string line;
		while (std::getline(in_, line)) {
			if (line == dotStr) break;

			auto p = Util::getNameAndValue(line);
			if (p.first == pitchStr) {
				config_.setPitch(p.second);
			} else if (p.first == rateStr) {
				config_.setRate(p.second);
			} else if (p.first == volumeStr) {
				config_.setVolume(p.second);
			} else if (p.first == punctuationModeStr) {
				config_.setPunctuationMode(p.second);
			} else if (p.first == spellingModeStr) {
				config_.setSpellingMode(p.second);
			} else if (p.first == capLetRecognStr) {
				config_.setCapitalLetterRecognition(p.second);
			} else if (p.first == voiceStr) {
				config_.setVoice(p.second);
			} else if (p.first == languageStr) {
				config_.setLanguage(p.second);
			} else if (p.first == synthesisVoiceStr) {
				config_.setSynthesisVoice(p.second);
			}
		}
	}
	setSynthCommand(COMMAND_SET, std::string{});

	sendResponse(respSetDoneStr);
}

void
ModuleController::handleSpeakCommand()
{
	sendResponse(respSpeakInitStr);

	std::ostringstream out;
	std::string line;
	while (std::getline(in_, line)) {
		if (line == dotStr) break;
		if (out.tellp() != 0) {
			out << ' ';
		}
		out << line;
	}
	std::string msg = out.str();

	Util::stripSSML(msg);

	state_ = STATE_SPEAKING;
	setSynthCommand(COMMAND_SPEAK, msg);
}

void
ModuleController::handleStopCommand()
{
	state_ = STATE_STOP_REQUESTED;
}

void
ModuleController::handleQuitCommand()
{
	setSynthCommand(COMMAND_QUIT, std::string());

	synthController_->wait();

	sendResponse(respQuitDoneStr);
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
			stream << respSynthInitFailStr_1 << msg << '\n' << respSynthInitFailStr_2;
			sendResponse(stream.str());
		} else {
			sendResponse(respSynthInitDoneStr);
		}
		break;
	case COMMAND_SPEAK:
		if (failed) {
			sendResponse(respSynthSpeakFailStr);
			state_ = STATE_IDLE;
		} else {
			sendResponse(respSynthSpeakDoneStr);
		}
		break;
	default:
		break;
	}
}

void
ModuleController::sendResponse(const std::string& msg)
{
	std::lock_guard<std::mutex> lock(responseMutex_);
	out_ << msg << std::endl;
}

void
ModuleController::sendBeginEvent()
{
	std::lock_guard<std::mutex> lock(responseMutex_);
	out_ << respBeginEventStr << std::endl;
}

void
ModuleController::sendEndEvent()
{
	responseMutex_.lock();
	out_ << respEndEventStr << std::endl;
	responseMutex_.unlock();

	state_ = STATE_IDLE;
}

void
ModuleController::sendStopEvent()
{
	responseMutex_.lock();
	out_ << respStopEventStr << std::endl;
	responseMutex_.unlock();

	state_ = STATE_IDLE;
}

void
ModuleController::getConfigCopy(ModuleConfiguration& config) {
	std::lock_guard<std::mutex> lock(configMutex_);
	config = config_;
}
