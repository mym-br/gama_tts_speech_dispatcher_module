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

#include <cstdlib>
#include <cstring> /* strcmp */
#include <iomanip>
#include <iostream>

#include "portaudiocpp/PortAudioCpp.hxx"

#include "ModuleController.h"

#define PROGRAM_NAME "sd_gama_tts"
#define PROGRAM_VERSION "0.1.6"



void
showUsage()
{
	std::cerr << "\nGamaTTS module " << PROGRAM_VERSION << " for Speech Dispatcher.\n\n";
	std::cerr << "Usage:\n"
			<< PROGRAM_NAME << " <config. file path>\n"
			<< "        Run module.\n"
			<< PROGRAM_NAME << " -o\n"
			<< "        List audio output devices.\n"
			<< std::endl;
}

void
showAudioOutputDevices()
{
	portaudio::System& sys = portaudio::System::instance();

	std::cout << "Audio output devices:\n";

	int i = 0;
	for (auto iter = sys.devicesBegin(); iter != sys.devicesEnd(); ++iter, ++i) {
		std::cout << std::setw(3) << i << ": " << '[' << iter->hostApi().name() << "] " << iter->name() << '\n';
	}

	std::cout << "Default output device: " << sys.defaultOutputDevice().index() << std::endl;
}



int
main(int argc, char* argv[])
{
	if (argc != 2) {
		showUsage();
		return EXIT_FAILURE;
	}

	portaudio::AutoSystem portaudio;

	if (std::strcmp(argv[1], "-o") == 0) {
		showAudioOutputDevices();
		return EXIT_SUCCESS;
	}

	ModuleController controller(std::cin, std::cout, argv[1]);
	controller.exec();

	return EXIT_SUCCESS;
}
