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

#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ModuleController.h"



namespace GS {
	namespace En {
		class PhoneticStringParser;
		class TextParser;
	}
	namespace TRM {
		class Tube;
	}
	namespace TRMControlModel {
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
	void init();
	void set();
	void speak();

	ModuleController& moduleController_;
	std::thread synthThread_;

	std::unique_ptr<GS::TRMControlModel::Model> model_;
	std::unique_ptr<GS::TRMControlModel::Controller> modelController_;
	std::unique_ptr<GS::En::TextParser> textParser_;
	std::unique_ptr<GS::En::PhoneticStringParser> phoneticStringParser_;
	std::unique_ptr<GS::TRM::Tube> tube_;

	ModuleController::CommandType commandType_;
	std::string commandMessage_;

	std::stringstream trmParamStream_;
	std::vector<float> outputBuffer_;

	ModuleConfiguration moduleConfig_;
	double defaultPitchOffset_;
};

#endif /* SYNTHESIZER_CONTROLLER_H_ */
