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

#include "Util.h"



namespace Util {

void
ignoreTags(std::string& msg)
{
	std::string::size_type prevPos = 0;
	std::string::size_type firstPos = 0;
	while ( prevPos < msg.size() && (firstPos = msg.find('<', prevPos)) != std::string::npos ) {
		if (firstPos == msg.size() - 1) {
			return;
		}
		std::string::size_type lastPos = msg.find('>', firstPos + 1);
		if (lastPos == std::string::npos) {
			prevPos = firstPos + 1;
		} else {
			for (std::string::size_type i = firstPos; i <= lastPos; ++i) {
				msg[i] = ' ';
			}
			prevPos = lastPos + 1;
		}
	}
}

std::pair<std::string, std::string>
getNameAndValue(const std::string& s)
{
	std::string::size_type pos = s.find('=');
	if (pos == std::string::npos || pos == s.size() - 1) {
		return std::make_pair(s, std::string());
	}
	return std::make_pair(s.substr(0, pos), s.substr(pos + 1, s.size() - pos - 1));
}

void
removeLF(std::string& msg)
{
	for (char& c : msg) {
		if (c == '\n') {
			c = ' ';
		}
	}
}

} // namespace Util
