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

#include <iostream>

#include "portaudiocpp/AutoSystem.hxx"

#include "ModuleController.h"



void
showUsage(const char* progName)
{
	std::cerr << "Usage: " << progName << " <config. file path>" << std::endl;
}

int
main(int argc, char* argv[])
{
	if (argc != 2) {
		showUsage(argv[0]);
		return -1;
	}

	portaudio::AutoSystem portaudio;

	ModuleController controller(std::cin, std::cout, argv[1]);
	controller.exec();

	return 0;
}