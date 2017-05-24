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

#ifndef MODULE_CONTROLLER_H_
#define MODULE_CONTROLLER_H_

#include <atomic>
#include <condition_variable>
#include <istream>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>

#include "ModuleConfiguration.h"



class SynthesizerController;

class ModuleController {
public:
	enum CommandType {
		COMMAND_NONE,
		COMMAND_INIT,
		COMMAND_SET,
		COMMAND_SPEAK,
		COMMAND_STOP,
		COMMAND_QUIT
	};
	enum State {
		STATE_IDLE,
		STATE_SPEAKING,
		STATE_STOP_REQUESTED
	};

	ModuleController(std::istream& in, std::ostream& out, const char* configFilePath);
	~ModuleController();

	// Main loop.
	void exec();

	// [commandMutex_] Called by ModuleController.
	void setSynthCommand(CommandType type, const std::string& message);

	// [commandMutex_] Called by SynthesizerController.
	void getSynthCommand(CommandType& type, std::string& message, bool wait=false);

	// [responseMutex_] Called by SynthesizerController.
	void setSynthCommandResult(CommandType type, bool failed, const std::string& msg);

	// [responseMutex_] Called by ModuleController.
	void sendResponse(const std::string& msg);

	// [responseMutex_] Called by SynthesizerController.
	void sendBeginEvent();

	// [responseMutex_] Called by SynthesizerController.
	void sendEndEvent();

	// [responseMutex_] Called by SynthesizerController.
	void sendStopEvent();

	// [configMutex_] Called by SynthesizerController.
	void getConfigCopy(ModuleConfiguration& config);

	// [atomic] Called by SynthesizerController.
	unsigned int state() const { return state_; }
private:
	void handleInitCommand();
	void handleAudioCommand();
	void handleLogLevelCommand();
	void handleSetCommand();
	void handleSpeakCommand();
	void handleStopCommand();
	void handleQuitCommand();

	std::atomic_uint state_;
	std::istream& in_;
	std::ostream& out_;
	std::string configFilePath_;
	ModuleConfiguration config_;
	std::mutex configMutex_;
	std::mutex commandMutex_;
	std::condition_variable commandSentCondition_; // command sent to SynthesizerController
	std::condition_variable commandReceivedCondition_; // command received by SynthesizerController
	std::mutex responseMutex_;

	bool newCommand_;
	CommandType commandType_;
	std::string commandMessage_;

	std::unique_ptr<SynthesizerController> synthController_;
};

#endif /* MODULE_CONTROLLER_H_ */
