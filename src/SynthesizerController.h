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

#ifndef SYNTHESIZER_CONTROLLER_H_
#define SYNTHESIZER_CONTROLLER_H_

#include <atomic>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "portaudiocpp/PortAudioCpp.hxx"

#include "ModuleController.h"



namespace GS {
	namespace En {
		class TextParser;
	}
	namespace VTM {
		class VocalTractModel;
	}
	namespace VTMControlModel {
		class Controller;
		class Model;
	}
}

class SynthesizerController {
public:
	SynthesizerController(ModuleController& moduleController);
	~SynthesizerController();

	void exec();
	void wait();
private:
	enum State {
		STATE_STOPPED,
		STATE_PLAYING,
		STATE_STOPPING
	};

	void init();
	void set();
	void speak();

	int portaudioCallback(const void* inputBuffer, void* paOutputBuffer, unsigned long framesPerBuffer,
				const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags);

	ModuleController& moduleController_;
	std::thread synthThread_;

	std::unique_ptr<GS::VTMControlModel::Model> model_;
	std::unique_ptr<GS::VTMControlModel::Controller> modelController_;
	std::unique_ptr<GS::En::TextParser> textParser_;
	std::unique_ptr<GS::VTM::VocalTractModel> vocalTractModel_;

	ModuleController::CommandType commandType_;
	std::string commandMessage_;

	std::stringstream vtmParamStream_;
	std::vector<float> audioBuffer_;

	ModuleConfiguration moduleConfig_;
	double defaultPitchOffset_;

	unsigned int audioBufferIndex_;
	unsigned int numInputChannels_;
	int audioOutputDeviceIndex_;

	std::atomic_uint state_;
	float fadeOutAmplitude_;
	float fadeOutDelta_;
};

#endif /* SYNTHESIZER_CONTROLLER_H_ */
